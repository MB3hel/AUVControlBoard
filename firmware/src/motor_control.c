/*
 * Copyright 2022 Marcus Behel
 * 
 * This file is part of AUVControlBoard-Firmware.
 * 
 * AUVControlBoard-Firmware is free software: you can redistribute it and/or modify it under the terms of the GNU 
 * General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * AUVControlBoard-Firmware is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even 
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with AUVControlBoard-Firmware. If not, see 
 * <https://www.gnu.org/licenses/>. 
 * 
 */

#include <motor_control.h>
#include <app.h>
#include <matrix.h>
#include <thruster.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <timers.h>
#include <cmdctrl.h>
#include <pid.h>
#include <math.h>
#include <debug.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <simulator.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MOTOR_WDOG_PERIOD_MS            1500

#define MC_RELSCALE_X                   (mc_relscale[0])
#define MC_RELSCALE_Y                   (mc_relscale[1])
#define MC_RELSCALE_Z                   (mc_relscale[2])
#define MC_RELSCALE_XROT                (mc_relscale[3])
#define MC_RELSCALE_YROT                (mc_relscale[4])
#define MC_RELSCALE_ZROT                (mc_relscale[5])

#define MAX(a, b)   (a > b ? a : b)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool mc_invert[8];                                      // Tracks thruster inversions

float mc_relscale[6];                                   // Relative DoF speeds

static float dof_matrix_arr[8*6];                       // Backing array for DoF matrix
static matrix dof_matrix;                               // DoF matrix

static float overlap_arrs[8][8];                        // Backing arrays for overlap vectors
static matrix overlap_vectors[8];                       // overlaps vectors

static bool motors_killed;                              // Motor (watchdog) state
static TimerHandle_t motor_wdog_timer;                  // Timer to implement motor watchdog

static SemaphoreHandle_t motor_mutex;                   // Ensures motor & watchdog access is thread safe

// PID controllers for SASSIST mode
static pid_controller_t xrot_pid, yrot_pid, zrot_pid, depth_pid;

// Current targets for PIDs
static quaternion_t pid_target_quat = {.w = 0, .x = 0, .y = 0, .z = 0};
static float pid_target_depth = -999.0f;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Initialization & Setup
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void mc_wdog_timeout(TimerHandle_t timer);

void mc_init(void){
    // Initialize matrices
    matrix_init_static(&dof_matrix, dof_matrix_arr, 8, 6);
    for(unsigned int i = 0; i < 8; ++i)
        matrix_init_static(&overlap_vectors[i], overlap_arrs[i], 8, 1);

    // Initialize pid controllers
    xrot_pid.kP = 0.0f;
    xrot_pid.kI = 0.0f;
    xrot_pid.kD = 0.0f;
    xrot_pid.min = -1.0f;
    xrot_pid.max = 1.0f;
    PID_RESET(xrot_pid);
    yrot_pid.kP = 0.0f;
    yrot_pid.kI = 0.0f;
    yrot_pid.kD = 0.0f;
    yrot_pid.min = -1.0f;
    yrot_pid.max = 1.0f;
    PID_RESET(yrot_pid);
    zrot_pid.kP = 0.0f;
    zrot_pid.kI = 0.0f;
    zrot_pid.kD = 0.0f;
    zrot_pid.min = -1.0f;
    zrot_pid.max = 1.0f;
    PID_RESET(zrot_pid);
    depth_pid.kP = 0.0f;
    depth_pid.kI = 0.0f;
    depth_pid.kD = 0.0f;
    depth_pid.min = -1.0f;
    depth_pid.max = 1.0f;
    PID_RESET(depth_pid);

    // Create required RTOS objects
    motor_mutex = xSemaphoreCreateMutex();
    motor_wdog_timer = xTimerCreate(
        "mwdog_tim",
        pdMS_TO_TICKS(MOTOR_WDOG_PERIOD_MS),
        pdFALSE,                                        // No autoreload = singleshot = becomes inactive after timeout
        NULL,
        mc_wdog_timeout
    );

    // Motors killed at startup
    motors_killed = true;

    // Default all motors to non-inverted
    for(unsigned int i = 0; i < 8; ++i){
        mc_invert[i] = false;
    }

    // Default to no dof scaling
    for(unsigned int i = 0; i < 6; ++i){
        mc_relscale[i] = 1.0f;
    }
}

void mc_set_dof_matrix(unsigned int thruster_num, float *row_data){
    matrix_set_row(&dof_matrix, thruster_num - 1, row_data);
}

void mc_recalc(void){
    // Called when done updating DoF matrix

    // Construct contribution  matrix (used to calculate overlap vectors)
    float contribution_arr[8*6];
    matrix contribution_matrix;
    matrix_init_static(&contribution_matrix, contribution_arr, 8, 6);
    for(size_t row = 0; row < contribution_matrix.rows; ++row){
        for(size_t col = 0; col < contribution_matrix.cols; ++col){
            float dof_item;
            matrix_get_item(&dof_item, &dof_matrix, row, col);
            matrix_set_item(&contribution_matrix, row, col, (dof_item != 0) ? 1 : 0);
        }
    }

    // Construct overlap vectors
    float rowdata[8];
    float v_arr[6];
    matrix v;
    matrix_init_static(&v, v_arr, 6, 1);
    for(size_t r = 0; r < contribution_matrix.rows; ++r){
        matrix_init_static(&overlap_vectors[r], overlap_arrs[r], contribution_matrix.rows, 1);

        // v = contribution_matrix row r transposed
        matrix_get_row(rowdata, &contribution_matrix, r);
        matrix_set_col(&v, 0, rowdata);

        matrix_mul(&overlap_vectors[r], &contribution_matrix, &v);

        for(size_t row = 0; row < overlap_vectors[r].rows; ++row){
            for(size_t col = 0; col < overlap_vectors[r].cols; ++col){
                float item;
                matrix_get_item(&item, &overlap_vectors[r], row, col);
                matrix_set_item(&overlap_vectors[r], row, col, (item != 0) ? 1 : 0);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Motor Watchdog
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void mc_wdog_timeout(TimerHandle_t timer){
    (void)timer;
    
    xSemaphoreTake(motor_mutex, portMAX_DELAY);
    motors_killed = true;
    if(sim_hijacked){
        for(unsigned int i = 0; i < 8; ++i){
            sim_speeds[i] = 0.0f;
        }
    }else{
        thruster_set((float[]){0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    }
    cmdctrl_mwdog_change(false);
    xSemaphoreGive(motor_mutex);
}

bool mc_wdog_feed(void){
    bool ret;
    xSemaphoreTake(motor_mutex, portMAX_DELAY);
    // If timer is inactive, this starts the timer
    // If timer is active, this resets the timer
    xTimerReset(motor_wdog_timer, portMAX_DELAY);
    ret = motors_killed;
    if(motors_killed)
        cmdctrl_mwdog_change(true);
    motors_killed = false;
    xSemaphoreGive(motor_mutex);
    return ret;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Extra math code (not in angles.c or matrix.c)
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Rotate a vector (x, y, z) buy a quaternion q
static inline void rotate_vector(float *dx, float *dy, float *dz, float sx, float sy, float sz, quaternion_t *q){
    quaternion_t qv;
    qv.w = 0.0f;
    qv.x = sx;
    qv.y = sy;
    qv.z = sz;    
    
    quaternion_t qconj;
    quat_conjugate(&qconj, q);

    quaternion_t qr;
    quat_multiply(&qr, &qv, &qconj);
    quat_multiply(&qr, q, &qr);
    *dx = qr.x;
    *dy = qr.y;
    *dz = qr.z;
}

// Rotate a vector (x, y, z) buy a the inverse of q
static inline void rotate_vector_inv(float *dx, float *dy, float *dz, float sx, float sy, float sz, quaternion_t *q){
    quaternion_t qv;
    qv.w = 0.0f;
    qv.x = sx;
    qv.y = sy;
    qv.z = sz;    
    
    quaternion_t qconj;
    quat_conjugate(&qconj, q);

    quaternion_t qr;
    quat_multiply(&qr, &qv, q);
    quat_multiply(&qr, &qconj, &qr);
    *dx = qr.x;
    *dy = qr.y;
    *dz = qr.z;
}

// Minimum rotation from a to b (as quaternion)
static inline void quat_diff(quaternion_t *dest, quaternion_t *a, quaternion_t *b){
    float dot;
    quat_dot(&dot, a, b);
    quaternion_t b_inv;
    quat_inverse(&b_inv, b);
    if(dot < 0.0){
        quat_multiply_scalar(&b_inv, &b_inv, -1.0f);
    }
    quat_multiply(dest, a, &b_inv);
}

// Quaternion rotation from vector a to vector b
static inline void quat_between(quaternion_t *dest, float ax, float ay, float az, float bx, float by, float bz){
    float dot = ax*bx + ay*by + az*bz;
    float a_len2 = ax*ax + ay*ay + az*az;
    float b_len2 = bx*bx + by*by + bz*bz;
    float p = sqrtf(a_len2 * b_len2);
    float cross_x = ay*bz - az*by;
    float cross_y = az*bx - ax*bz;
    float cross_z = ax*by - ay*bx;
    if(dot / p == -1){
        // 180 degree rotation
        dest->w = 0.0f;
        dest->x = cross_x;
        dest->y = cross_y;
        dest->z = cross_z;
    }else{
        dest->w = dot + p;
        dest->x = cross_x;
        dest->y = cross_y;
        dest->z = cross_z;
    }
    quat_normalize(dest, dest);
}

// Twist part of swing-twist decomposition
// calculates twist of q around axis described by vector d = <d_x, d_y, d_z>
// Note that d must be normalized!!!
static inline void quat_twist(quaternion_t *twist, quaternion_t *q, float d_x, float d_y, float d_z){
    // r = axis of rotation = <q.x, q.y, q.z>
    float r_x = q->x;
    float r_y = q->y;
    float r_z = q->z;

    // dot = <r, d> (dot product)
    float dot = r_x*d_x + r_y*d_y + r_z*d_z;

    // p = projection or r on d = ((<r, d>) / (|d|^2))d
    // p = <r,d>d if d is unit vector (as it is assumed to be here)
    float p_x = dot * d_x;
    float p_y = dot * d_y;
    float p_z = dot * d_z;

    // twist = (w=q.w, x=p.x, y=p.y, z=p.z).normalized()
    twist->w = q->w;
    twist->x = p_x;
    twist->y = p_y;
    twist->z = p_z;
    quat_normalize(twist, twist);

    // However, if <r, d> is negative, the twist will be opposite the axis of rotation
    // Thus, negate the quaternion so twist's axis points in same direction as d
    // and it's angle will be the expected sign
    if(dot < 0.0f){
        quat_multiply_scalar(twist, twist, -1.0f);
    }
}

// Restrict angle to between -PI and PI radians or -180.0 and 180.0 degrees
static inline float restrict_angle(float angle, bool isdeg){
    if(isdeg){
        while(angle > 180.0f){
            angle -= 360.0f;
        }
        while(angle < -180.0f){
            angle += 360.0f;
        }
    }else{
        while(angle > M_PI){
            angle -= 2.0f * M_PI;
        }
        while(angle < -M_PI){
            angle += 2.0f * M_PI;
        }
    }
    return angle;
}

// Get alternate (equivalent, but improper) euler angle
// Dest uses same unit as src
static inline void euler_alt(euler_t *dest, euler_t *src){
    dest->is_deg = src->is_deg;
    if(src->is_deg){
        dest->pitch = 180.0f - src->pitch;
        dest->roll = src->roll - 180.0f;
        dest->yaw = src->yaw - 180.0f;
    }else{
        dest->pitch = M_PI - src->pitch;
        dest->roll = src->roll - M_PI;
        dest->yaw = src->yaw - M_PI;
    }
    dest->pitch = restrict_angle(dest->pitch, dest->is_deg);
    dest->roll = restrict_angle(dest->roll, dest->is_deg);
    dest->yaw = restrict_angle(dest->yaw, dest->is_deg);
}

// Get magnitude of largest magnitude element of vector
static inline float vec_max_mag(float x, float y, float z){
    return MAX(fabsf(x), MAX(fabsf(y), fabsf(z)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Motor control Support
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Use mc_relscale factors to scale down speeds as needed
// d[x, y, z] = destination (adjusted speeds)
// s[x, y, z] = source (speeds)
static inline void mc_downscale_reldof(float *dx, float *dy, float *dz, float sx, float sy, float sz, bool angular){
    // Get scale factors
    float scale_x = angular ? MC_RELSCALE_XROT : MC_RELSCALE_X;
    float scale_y = angular ? MC_RELSCALE_YROT : MC_RELSCALE_Y;
    float scale_z = angular ? MC_RELSCALE_ZROT : MC_RELSCALE_Z;

    // If a component of the input is zero, no reason to downscale considering that component
    if(fabsf(sx) < 1e-4){
        scale_x = 0.0f;
    }
    if(fabsf(sy) < 1e-4){
        scale_y = 0.0f;
    }
    if(fabsf(sz) == 1e-4){
        scale_z = 0.0f;
    }

    // Rebalance scale factors so largest is 1.0f
    float maxscale = MAX(scale_x, MAX(scale_y, scale_z));
    scale_x /= maxscale;
    scale_y /= maxscale;
    scale_z /= maxscale;

    // Do scaling
    *dx = sx * scale_x;
    *dy = sy * scale_y;
    *dz = sz * scale_z;
}

// Scale s=<sx, sy, sz> so that largest element is v (assumed that s is unit vector)
// Store result in d=<dx,dy,dz>
static inline void mc_upscale_vec(float  *dx, float *dy, float *dz, float sx, float sy, float sz, float v){
    if(fabsf(v) < 1e-4){
        *dx = 0.0f;
        *dy = 0.0f;
        *dz = 0.0f;
    }else{
        float max_mag = vec_max_mag(sx, sy, sz);
        *dx = (sx / max_mag) * fabsf(v);
        *dy = (sy / max_mag) * fabsf(v);
        *dz = (sz / max_mag) * fabsf(v);
    }
}

// Ensure all elements of s=<sx, sy, sz> have a magnitude of less than 1.0
// Downscale proportionally to achieve this if necessary
static inline void mc_downscale_if_needed(float *dx, float *dy, float *dz, float sx, float sy, float sz){
    float maxmag = vec_max_mag(sx, sy, sz);
    if(maxmag > 1.0f){
        *dx = sx / maxmag;
        *dy = sy / maxmag;
        *dz = sz / maxmag;
    }else{
        *dx = sx;
        *dy = sy;
        *dz = sz;
    }
}

// Given the vehicle's current orientation, construct a gravity vector
// Then, get the angle between the gravity vector and "base" gravity vector
// Return the angle from the base gravity vector to the current one
static inline void mc_grav_rot(quaternion_t *qrot, quaternion_t *qcurr){
    // Gravity vector component equations come from applying rotation matrix form to <0, 0, -1>
    float grav_x = 2.0f * (-qcurr->x*qcurr->z + qcurr->w*qcurr->y);
    float grav_y = 2.0f * (-qcurr->w*qcurr->x - qcurr->y*qcurr->z);
    float grav_z = -qcurr->w*qcurr->w + qcurr->x*qcurr->x + qcurr->y*qcurr->y - qcurr->z*qcurr->z;
    
    // Make sure this is a unit vector
    float grav_mag = sqrtf(grav_x*grav_x + grav_y*grav_y + grav_z*grav_z);
    grav_x /= grav_mag;
    grav_y /= grav_mag;
    grav_z /= grav_mag;

    // Get angle from <0, 0, -1> to <grav_x, grav_y, grav_z>
    quat_between(qrot, 0.0f, 0.0f, -1.0f, grav_x, grav_y, grav_z);
}

// Convert the given quaternion to euler angles
// Construct the alternate euler representation (improper)
// Return the euler angle with minimal roll component
static inline void mc_baseline_euler(euler_t *ebase, quaternion_t *qcurr){
    euler_t e_orig, e_alt;
    quat_to_euler(&e_orig, qcurr);
    euler_alt(&e_alt, &e_orig);
    *ebase = (fabsf(e_orig.roll) < fabsf(e_alt.roll)) ? e_orig : e_alt;
}

// Split euler e into three quaternions for pitch, roll, and yaw rotations seperately
// Current orientation of vehicle qcurr = qyaw * qpitch * qroll using the Z-X'-Y'' convention
static inline void mc_euler_to_split_quat(quaternion_t *qpitch, quaternion_t *qroll, quaternion_t *qyaw, euler_t e){
    euler_t e_pitch = {.is_deg = e.is_deg, .pitch = e.pitch, .roll = 0.0f, .yaw = 0.0f};
    euler_t e_roll = {.is_deg = e.is_deg, .pitch = 0.0f, .roll = e.roll, .yaw = 0.0f};
    euler_t e_yaw = {.is_deg = e.is_deg, .pitch = 0.0f, .roll = 0.0f, .yaw = e.yaw};
    euler_to_quat(qpitch, &e_pitch);
    euler_to_quat(qroll, &e_roll);
    euler_to_quat(qyaw, &e_yaw);
}



void mc_sassist_tune_xrot(float kp, float ki, float kd, float limit, bool invert){
    xrot_pid.kP = kp;
    xrot_pid.kI = ki;
    xrot_pid.kD = kd;
    xrot_pid.max = limit;
    xrot_pid.min = -limit;
    xrot_pid.invert = invert;
}

void mc_sassist_tune_yrot(float kp, float ki, float kd, float limit, bool invert){
    yrot_pid.kP = kp;
    yrot_pid.kI = ki;
    yrot_pid.kD = kd;
    yrot_pid.max = limit;
    yrot_pid.min = -limit;
    yrot_pid.invert = invert;
}

void mc_sassist_tune_zrot(float kp, float ki, float kd, float limit, bool invert){
    zrot_pid.kP = kp;
    zrot_pid.kI = ki;
    zrot_pid.kD = kd;
    zrot_pid.max = limit;
    zrot_pid.min = -limit;
    zrot_pid.invert = invert;
}

void mc_sassist_tune_depth(float kp, float ki, float kd, float limit, bool invert){
    depth_pid.kP = kp;
    depth_pid.kI = ki;
    depth_pid.kD = kd;
    depth_pid.max = limit;
    depth_pid.min = -limit;
    depth_pid.invert = invert;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Motor control
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void mc_set_raw(float *speeds){
    xSemaphoreTake(motor_mutex, portMAX_DELAY);   

    // Don't allow speed set while motors are killed
    if(!motors_killed){
        // Apply thruster inversions
        for(unsigned int i = 0; i < 8; ++i){
            if(mc_invert[i])
                speeds[i] *= -1;
        }

        // Actually set thruster speeds
        if(sim_hijacked){
            for(unsigned int i = 0; i < 8; ++i){
                sim_speeds[i] = speeds[i];
            }
        }else{
            thruster_set(speeds);
        }
    }

    xSemaphoreGive(motor_mutex);
}

void mc_set_local(float x, float y, float z, float xrot, float yrot, float zrot){
    float target_arr[6];
    matrix target;
    matrix_init_static(&target, target_arr, 6, 1);
    matrix_set_col(&target, 0, (float[]){x, y, z, xrot, yrot, zrot});

    // Limit input speeds to correct range
    for(size_t i = 0; i < 6; ++i){
        if(target_arr[i] > 1.0)
            target_arr[i] = 1.0;
        if(target_arr[i] < -1.0)
            target_arr[i] = -1.0;
    }

    float speed_arr[8];
    matrix speed_vec;
    matrix_init_static(&speed_vec, speed_arr, 8, 1);

    // Base speed calculation
    matrix_mul(&speed_vec, &dof_matrix, &target);

    // Scale motor speeds down as needed
    while(true){
        size_t idxrow, idxcol;
        float mval;
        matrix_absmax(&mval, &idxrow, &idxcol, &speed_vec);
        if(mval <= 1)
            break;
        for(size_t i = 0; i < overlap_vectors[idxrow].rows; ++i){
            float cval;
            matrix_get_item(&cval, &overlap_vectors[idxrow], i, 0);
            if(cval == 1){
                matrix_get_item(&cval, &speed_vec, i, 0);
                cval /= mval;
                matrix_set_item(&speed_vec, i, 0, cval);
            }
        }
    }    

    // Speed array already contains motor speeds in order
    // Because dof matrix rows are in order
    mc_set_raw(speed_arr);
}

void mc_set_global(float x, float y, float z, float pitch_spd, float roll_spd, float yaw_spd, quaternion_t curr_quat){
    // -----------------------------------------------------------------------------------------------------------------
    // Translation
    // -----------------------------------------------------------------------------------------------------------------
    // Use gravity vectors to construct pitch and roll compensation quaternion
    quaternion_t qrot;
    mc_grav_rot(&qrot, &curr_quat);

    // Compute each translation component separately and upscale
    // Upscaling ensures largest magnitude of each component equals the speed of the component
    // Note that tx, ty, and tz vectors are in vehicle DoFs
    float tx_x, tx_y, tx_z;
    rotate_vector(&tx_x, &tx_y, &tx_z, x, 0.0f, 0.0f, &qrot);
    mc_upscale_vec(&tx_x, &tx_y, &tx_z, tx_x, tx_y, tx_z, x);

    float ty_x, ty_y, ty_z;
    rotate_vector(&ty_x, &ty_y, &ty_z, 0.0f, y, 0.0f, &qrot);
    mc_upscale_vec(&ty_x, &ty_y, &ty_z, ty_x, ty_y, ty_z, y);

    float tz_x, tz_y, tz_z;
    rotate_vector(&tz_x, &tz_y, &tz_z, 0.0f, 0.0f, z, &qrot);
    mc_upscale_vec(&tz_x, &tz_y, &tz_z, tz_x, tz_y, tz_z, z);
    
    // Combine each translation component
    float lx = tx_x + ty_x + tz_x;
    float ly = tx_y + ty_y + tz_y;
    float lz = tx_z + ty_z + tz_z;
    
    // Compensate for differences in vehicle speed in different DoFs
    mc_downscale_reldof(&lx, &ly, &lz, lx, ly, lz, false);

    // Proportionally scale translation speeds so all have magnitude less than 1.0
    mc_downscale_if_needed(&lx, &ly, &lz, lx, ly, lz);
    // -----------------------------------------------------------------------------------------------------------------


    // -----------------------------------------------------------------------------------------------------------------
    // Rotation
    // -----------------------------------------------------------------------------------------------------------------
    // Get split pitch, roll, and yaw quaternions
    quaternion_t q_pitch, q_roll, q_yaw;
    euler_t e_base;
    mc_baseline_euler(&e_base, &curr_quat);
    mc_euler_to_split_quat(&q_pitch, &q_roll, &q_yaw, e_base);

    // Compute each rotation component seperately
    // s_roll, s_pitch, s_yaw are in zero rotation frame
    // Ie: s_yaw = <0, 0, yaw_spd> means rotation about world z to change vehicle yaw
    // w_roll, w_pitch, w_yaw  are angular velocities in current vehicle frame

    // Roll is already in vehicle frame (last rotation applied)
    float w_roll_y = roll_spd;

    // w_pitch = q_roll_inv * s_pitch * q_roll
    // s_pitch = <pitch_spd, 0, 0>
    // Note: In gimbal lock scenarios (pitch = +/-90 deg) changing vehicle pitch
    //       is ambiguous. This will use the "zero roll" solution
    //       However, this may cause discontinuous motion when moving through gimbal lock positions
    float s_pitch_x = pitch_spd, s_pitch_y = 0.0f, s_pitch_z = 0.0f;
    float w_pitch_x, w_pitch_y, w_pitch_z;
    rotate_vector_inv(&w_pitch_x, &w_pitch_y, &w_pitch_z, s_pitch_x, s_pitch_y, s_pitch_z, &q_roll);

    // w_yaw = q_roll_inv * q_pitch_inv * s_yaw * q_pitch * q_roll
    // s_yaw = <0, 0, yaw_spd>
    float s_yaw_x = 0.0f, s_yaw_y = 0.0f, s_yaw_z = yaw_spd;
    float w_yaw_x, w_yaw_y, w_yaw_z;
    rotate_vector_inv(&w_yaw_x, &w_yaw_y, &w_yaw_z, s_yaw_x, s_yaw_y, s_yaw_z, &q_pitch);
    rotate_vector_inv(&w_yaw_x, &w_yaw_y, &w_yaw_z, w_yaw_x, w_yaw_y, w_yaw_z, &q_roll);

    // Scale up each group as needed
    // Note: not needed for w_roll because only one component
    mc_upscale_vec(&w_pitch_x, &w_pitch_y, &w_pitch_z, w_pitch_x, w_pitch_y, w_pitch_z, pitch_spd);
    mc_upscale_vec(&w_yaw_x, &w_yaw_y, &w_yaw_z, w_yaw_x, w_yaw_y, w_yaw_z, yaw_spd);

    // Calculate total rotations in local DoFs
    // These sums may exceed 1.0
    float xrot = w_pitch_x  + 0.0f      + w_yaw_x;
    float yrot = w_pitch_y  + w_roll_y  + w_yaw_y;
    float zrot = w_pitch_z  + 0.0f      + w_yaw_z;

    // Compensate for differences in vehicle speed in different DoFs
    mc_downscale_reldof(&xrot, &yrot, &zrot, xrot, yrot, zrot, true);

    // Proportionally scale rotation speeds so all have magnitude less than 1.0
    mc_downscale_if_needed(&xrot, &yrot, &zrot, xrot, yrot, zrot);
    // -----------------------------------------------------------------------------------------------------------------

    // Set speeds
    mc_set_local(lx, ly, lz, xrot, yrot, zrot);
}

void mc_set_sassist(float x, float y, float yaw_spd,
        euler_t target_euler,
        float target_depth,
        quaternion_t curr_quat,
        float curr_depth,
        bool yaw_target){

    // Baseline angular motion due to yaw speed (if yaw target not in use)
    float base_xrot = 0.0f;
    float base_yrot = 0.0f;
    float base_zrot = 0.0f;

    // Use gravity vectors to construct pitch and roll compensation quaternion
    // Used for x/y translation, but also for yaw speed if not using yaw target PID
    quaternion_t qrot;
    mc_grav_rot(&qrot, &curr_quat);

    // -----------------------------------------------------------------------------------------------------------------
    // Handle Open-Loop Yaw control (no yaw PID / yaw target, but instead a yaw speed)
    // -----------------------------------------------------------------------------------------------------------------
    if(!yaw_target){
        // Get twist part of current quaternion about z axis
        quaternion_t curr_twist;
        quat_twist(&curr_twist, &curr_quat, 0, 0, 1);

        // Extract yaw from twist quaternion.
        // Doing this with twist quat ensures the extracted yaw is not "offset by 180"
        // as it could be with converting curr_quat directly to euler angles
        // Eg: Consider pitch = 115, roll = 0, yaw = 0
        //     This is improper. The proper representation is pitch = 65, roll = -180, yaw = -180
        //     But we need a yaw of zero, not 180!
        float curr_yaw = atan2f(-2.0f * (curr_twist.x*curr_twist.y - curr_twist.w*curr_twist.z), 1.0f - 2.0f * (curr_twist.x*curr_twist.x + curr_twist.z*curr_twist.z));
        curr_yaw *= 180.0f / M_PI;
        target_euler.yaw = curr_yaw;

        // Same as yaw speed in GLOBAL mode
        rotate_vector(&base_xrot, &base_yrot, &base_zrot, 0.0f, 0.0f, yaw_spd, &qrot);
        mc_upscale_vec(&base_xrot, &base_yrot, &base_zrot, base_xrot, base_yrot, base_zrot, yaw_spd);
    }
    // -----------------------------------------------------------------------------------------------------------------


    // -----------------------------------------------------------------------------------------------------------------
    // Rotation
    // -----------------------------------------------------------------------------------------------------------------
    // Convert target orientation to quaternion
    quaternion_t target_quat;
    euler_to_quat(&target_quat, &target_euler);

    // Get shortest quaternion from current to target   
    // q_d is shortest rotation from q_c (curr_quat) to q_t (taret_quat) 
    float dot;
    quat_dot(&dot, &curr_quat, &target_quat);
    quaternion_t q_c_conj;
    if(dot < 0.0f){
        // If dot is negative, need to flit curr_quat before conj
        // to ensure shortest path
        quaternion_t temp;
        temp.w = curr_quat.w;
        temp.x = curr_quat.x;
        temp.y = curr_quat.y;
        temp.z = curr_quat.z;
        quat_multiply_scalar(&temp, &temp, -1.0f);
        quat_conjugate(&q_c_conj, &temp);
    }else{
        quat_conjugate(&q_c_conj, &curr_quat);
    }
    quaternion_t q_d;
    quat_multiply(&q_d, &q_c_conj, &target_quat);

    // Convert q_d to axis angle
    float mag = sqrtf(q_d.x*q_d.x + q_d.y*q_d.y + q_d.z*q_d.z);
    float theta = 2.0f * atan2f(mag, q_d.w);
    float ax = q_d.x;
    float ay = q_d.y;
    float az = q_d.z;
    if(mag > 0.001f){
        ax /= mag;
        ay /= mag;
        az /= mag;
    }

    // Angular velocity "errors" (for PIDs) are angle * axis
    float e_x = ax * theta;
    float e_y = ay * theta;
    float e_z = az * theta;

    // Reset PID controllers when targets change significantlly
    if(fabsf(pid_target_depth - target_depth) > 0.01){
        PID_RESET(depth_pid);
    }
    if(fabsf(target_quat.w - pid_target_quat.w) > 0.01 || 
            fabsf(target_quat.x - pid_target_quat.x) > 0.01 ||
            fabsf(target_quat.y - pid_target_quat.y) > 0.01 ||
            fabsf(target_quat.z - pid_target_quat.z) > 0.01){
        PID_RESET(xrot_pid);
        PID_RESET(yrot_pid);
        PID_RESET(zrot_pid);
    }

    // Use PID controllers to calculate current outputs
    float z = -pid_calculate(&depth_pid, curr_depth - target_depth);
    float xrot = pid_calculate(&xrot_pid, e_x);
    float yrot = pid_calculate(&yrot_pid, e_y);
    float zrot = pid_calculate(&zrot_pid, e_z);

    // Store old targets (used to determine when to reset PIDs)
    pid_target_depth = target_depth;
    pid_target_quat = target_quat;

    // Need to add PID outputs to base speeds of vehicle
    // Base speeds came from rotation of yaw speed onto vehicle basis
    xrot += base_xrot;
    yrot += base_yrot;
    zrot += base_zrot;

    // Compensate for differences in vehicle speed in different DoFs
    mc_downscale_reldof(&xrot, &yrot, &zrot, xrot, yrot, zrot, true);

    // Proportionally scale rotation speeds so all have magnitude less than 1.0
    mc_downscale_if_needed(&xrot, &yrot, &zrot, xrot, yrot, zrot);
    // -----------------------------------------------------------------------------------------------------------------


    // -----------------------------------------------------------------------------------------------------------------
    // Translation (same as global mode here)
    // -----------------------------------------------------------------------------------------------------------------
    // Compute each translation component separately and upscale
    // Upscaling ensures largest magnitude of each component equals the speed of the component
    // Note that tx, ty, and tz vectors are in vehicle DoFs
    float tx_x, tx_y, tx_z;
    rotate_vector(&tx_x, &tx_y, &tx_z, x, 0.0f, 0.0f, &qrot);
    mc_upscale_vec(&tx_x, &tx_y, &tx_z, tx_x, tx_y, tx_z, x);

    float ty_x, ty_y, ty_z;
    rotate_vector(&ty_x, &ty_y, &ty_z, 0.0f, y, 0.0f, &qrot);
    mc_upscale_vec(&ty_x, &ty_y, &ty_z, ty_x, ty_y, ty_z, y);

    float tz_x, tz_y, tz_z;
    rotate_vector(&tz_x, &tz_y, &tz_z, 0.0f, 0.0f, z, &qrot);
    mc_upscale_vec(&tz_x, &tz_y, &tz_z, tz_x, tz_y, tz_z, z);
    
    // Combine each translation component
    float lx = tx_x + ty_x + tz_x;
    float ly = tx_y + ty_y + tz_y;
    float lz = tx_z + ty_z + tz_z;
    
    // Compensate for differences in vehicle speed in different DoFs
    mc_downscale_reldof(&lx, &ly, &lz, lx, ly, lz, false);

    // Proportionally scale translation speeds so all have magnitude less than 1.0
    mc_downscale_if_needed(&lx, &ly, &lz, lx, ly, lz);
    // -----------------------------------------------------------------------------------------------------------------


    // Target motion now relative to the robot's axes
    mc_set_local(lx, ly, lz, xrot, yrot, zrot);
}

void mc_set_dhold(float x, float y, float pitch_spd, float roll_spd, float yaw_spd, float target_depth, quaternion_t curr_quat, float curr_depth){
    if(fabsf(pid_target_depth - target_depth) > 0.01){
        PID_RESET(depth_pid);
    }
    float z = -pid_calculate(&depth_pid, curr_depth - target_depth);
    pid_target_depth = target_depth;
    mc_set_global(x, y, z, pitch_spd, roll_spd, yaw_spd, curr_quat);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

// Get element of vector with maximum magnitude
static inline float vec_max_mag(float x, float y, float z){
    #define MAX(a, b)   (a > b ? a : b)
    return MAX(fabsf(x), MAX(fabsf(y), fabsf(z)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Motor control
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
    #define MAX(a, b)   ((a > b) ? a : b)
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
    // Use gravity vector to transform translation to 
    float grav_x = 2.0f * (-curr_quat.x*curr_quat.z + curr_quat.w*curr_quat.y);
    float grav_y = 2.0f * (-curr_quat.w*curr_quat.x - curr_quat.y*curr_quat.z);
    float grav_z = -curr_quat.w*curr_quat.w + curr_quat.x*curr_quat.x + curr_quat.y*curr_quat.y - curr_quat.z*curr_quat.z;
    float grav_mag = sqrtf(grav_x*grav_x + grav_y*grav_y + grav_z*grav_z);
    grav_x /= grav_mag;
    grav_y /= grav_mag;
    grav_z /= grav_mag;
    quaternion_t qrot;
    quat_between(&qrot, grav_x, grav_y, grav_z, 0.0f, 0.0f, -1.0f);
    quat_inverse(&qrot, &qrot);
    rotate_vector(&x, &y, &z, x, y, z, &qrot);
    
    // TODO: Translation relscale

    // TODO: Translation upscaling to match input speeds???

    // Calculate split pitch, roll, and yaw quaternions
    // Required to calculate speeds to change vehicle pitch and yaw
    euler_t e_orig, e_alt;
    quat_to_euler(&e_orig, &curr_quat);
    euler_alt(&e_alt, &e_orig);
    float e_orig_sum = fabsf(e_orig.roll);
    float e_alt_sum = fabsf(e_alt.roll);
    euler_t *e_min = (e_orig_sum < e_alt_sum) ? &e_orig : &e_alt;
    euler_t e_pitch = {.is_deg = e_min->is_deg, .pitch = e_min->pitch, .roll = 0.0f, .yaw = 0.0f};
    euler_t e_roll = {.is_deg = e_min->is_deg, .pitch = 0.0f, .roll = e_min->roll, .yaw = 0.0f};
    euler_t e_yaw = {.is_deg = e_min->is_deg, .pitch = 0.0f, .roll = 0.0f, .yaw = e_min->yaw};
    quaternion_t q_pitch, q_roll, q_yaw;
    euler_to_quat(&q_pitch, &e_pitch);
    euler_to_quat(&q_roll, &e_roll);
    euler_to_quat(&q_yaw, &e_yaw);

    // w_roll = s_roll = <0, roll_spd, 0>
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
    float maxmag = vec_max_mag(xrot, yrot, zrot);
    maxmag = (maxmag > 1.0f) ? maxmag : 1.0f;
    xrot /= maxmag;
    yrot /= maxmag;
    zrot /= maxmag;

    // Set speeds
    mc_set_local(x, y, z, xrot, yrot, zrot);
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

void mc_set_sassist(float x, float y, float yaw_spd,
        euler_t target_euler,
        float target_depth,
        quaternion_t curr_quat,
        float curr_depth,
        bool yaw_target){

    float base_xrot = 0.0f;
    float base_yrot = 0.0f;
    float base_zrot = 0.0f;

    if(!yaw_target){
        // euler_t curr_euler;
        // quat_to_euler(&curr_euler, &curr_quat);
        // euler_rad2deg(&curr_euler, &curr_euler);
        // target_euler.yaw = curr_euler.yaw;

        quaternion_t curr_twist;
        quat_twist(&curr_twist, &curr_quat, 0, 0, 1);
        float curr_yaw = atan2f(-2.0f * (curr_twist.x*curr_twist.y - curr_twist.w*curr_twist.z), 1.0f - 2.0f * (curr_twist.x*curr_twist.x + curr_twist.z*curr_twist.z));
        curr_yaw *= 180.0f / M_PI;

        target_euler.yaw = curr_yaw;


        quaternion_t curr_swing;
        quaternion_t curr_twist_conj;
        quat_conjugate(&curr_twist_conj, &curr_twist);
        quat_multiply(&curr_swing, &curr_quat, &curr_twist_conj);

        // Same transform as global mode translation
        float grav_x = 2.0f * (-curr_quat.x*curr_quat.z + curr_quat.w*curr_quat.y);
        float grav_y = 2.0f * (-curr_quat.w*curr_quat.x - curr_quat.y*curr_quat.z);
        float grav_z = -curr_quat.w*curr_quat.w + curr_quat.x*curr_quat.x + curr_quat.y*curr_quat.y - curr_quat.z*curr_quat.z;
        float grav_mag = sqrtf(grav_x*grav_x + grav_y*grav_y + grav_z*grav_z);
        grav_x /= grav_mag;
        grav_y /= grav_mag;
        grav_z /= grav_mag;
        quaternion_t qrot;
        quat_between(&qrot, grav_x, grav_y, grav_z, 0.0f, 0.0f, -1.0f);
        quat_inverse(&qrot, &qrot);
        rotate_vector(&base_xrot, &base_yrot, &base_zrot, 0.0f, 0.0f, yaw_spd, &qrot);
    }

    // Convert target to quaternion
    quaternion_t target_quat;
    euler_to_quat(&target_quat, &target_euler);


    // TODO: Test this. Dev math.
    float dot;
    quat_dot(&dot, &curr_quat, &target_quat);

    quaternion_t q_c_conj;
    if(dot < 0.0f){
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

    float mag = sqrtf(q_d.x*q_d.x + q_d.y*q_d.y + q_d.z*q_d.z);
    float theta = 2.0f * atan2f(mag, q_d.w);
    float ax = q_d.x;
    float ay = q_d.y;
    float az = q_d.z;
    if(mag > 0.001f){
        ax /= mag;
        ay /= mag;
        az /= mag;
    }else{
        // TODO: Maybe? Probably better to not?
        // ax = 0.0f;
        // ay = 0.0f;
        // az = 0.0f;
    }

    // TODO: This feels like it won't quite work in 3D
    // maybe it does, but need to work out the math fully
    // |theta| > 180.0 deg means we're trying to rotate the long way around
    // This is not ideal.
    // So fix it.
    // if(theta > (float)M_PI){
    //     theta -= M_PI;
    //     theta -= M_PI;
    // }
    // if(theta < -((float)M_PI)){
    //     theta += M_PI;
    //     theta += M_PI;
    // }

    float ww_x = ax * theta;
    float ww_y = ay * theta;
    float ww_z = az * theta;

    quaternion_t q_ww;
    q_ww.w = 0.0f;
    q_ww.x = ww_x;
    q_ww.y = ww_y;
    q_ww.z = ww_z;

    // quaternion_t q_wv;
    // quat_multiply(&q_wv, &curr_quat, &q_ww);

    // float wv_x = q_wv.x;
    // float wv_y = q_wv.y;
    // float wv_z = q_wv.z;

    float wv_x = ww_x;
    float wv_y = ww_y;
    float wv_z = ww_z;

    // Use PID controllers to calculate current outputs
    float z = -pid_calculate(&depth_pid, curr_depth - target_depth);
    float xrot = pid_calculate(&xrot_pid, wv_x);
    float yrot = pid_calculate(&yrot_pid, wv_y);
    float zrot = pid_calculate(&zrot_pid, wv_z);

    if(!yaw_target){
        // Need to add PID outputs to base speeds of vehicle
        // Base speeds came from rotation of yaw speed onto vehicle basis
        xrot += base_xrot;
        yrot += base_yrot;
        zrot += base_zrot;

        // Proportionally scale speeds so all have magnitude less than 1.0
        float maxmag = 1.0f;
        float abspitch = fabsf(xrot);
        if(abspitch > maxmag){
            maxmag = abspitch;
        }
        float absroll = fabsf(yrot);
        if(absroll > maxmag){
            maxmag = absroll;
        }
        float absyaw = fabsf(zrot);
        if(absyaw > maxmag){
            maxmag = absyaw;
        }
        xrot /= maxmag;
        yrot /= maxmag;
        zrot /= maxmag;
    }

    // Apply same rotation as in GLOBAL mode to translation target
    float grav_x = 2.0f * (-curr_quat.x*curr_quat.z + curr_quat.w*curr_quat.y);
    float grav_y = 2.0f * (-curr_quat.w*curr_quat.x - curr_quat.y*curr_quat.z);
    float grav_z = -curr_quat.w*curr_quat.w + curr_quat.x*curr_quat.x + curr_quat.y*curr_quat.y - curr_quat.z*curr_quat.z;
    float grav_mag = sqrtf(grav_x*grav_x + grav_y*grav_y + grav_z*grav_z);
    grav_x /= grav_mag;
    grav_y /= grav_mag;
    grav_z /= grav_mag;
    quaternion_t qrot;
    quat_between(&qrot, grav_x, grav_y, grav_z, 0.0f, 0.0f, -1.0f);
    quat_inverse(&qrot, &qrot);
    rotate_vector(&x, &y, &z, x, y, z, &qrot);

    // TODO: Apply relscale in sassist

    // Target motion now relative to the robot's axes
    mc_set_local(x, y, z, xrot, yrot, zrot);
}

void mc_set_dhold(float x, float y, float pitch_spd, float roll_spd, float yaw_spd, float target_depth, quaternion_t curr_quat, float curr_depth){
    // TODO: Redefine
}

// TODO: local sassist

// TODO: local depth hold

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

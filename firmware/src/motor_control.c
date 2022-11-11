/**
 * @file motor_control.c
 * @author Marcus Behel
 */

#include <motor_control.h>
#include <matrix.h>
#include <stdint.h>
#include <timers.h>


#define WD_DISABLE_COUNT         15                     // 15 * 100ms = 1500ms

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static float dof_matrix_arr[8*6];                       // Backing array for dof matrix
static matrix dof_matrix;                               // dof matrix

static float overlap_arrs[8][8];                        // Backing arrays for overlap vectors
static matrix overlap_vectors[8];                       // overlaps vectors

static uint16_t motor_wd_count;                         // Motor watchdog counter

bool motor_control_tinv[8];                             // Thruster inversion status (true = inverted)

static pid_t depth_pid;                                 // Depth hold PID controller
static pid_t pitch_pid;                                 // Pitch hold PID controller
static pid_t roll_pid;                                  // Roll hold PID controller


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int skew3(matrix *outmat, matrix *invec){
    if(outmat->rows != 3 || outmat->cols != 3)
        return MAT_ERR_SIZE;
    float v[3];
    if(invec->rows == 1){
        if(invec->cols != 3)
            return MAT_ERR_SIZE;
        matrix_get_row(&v[0], invec, 0);
    }else if(invec->cols == 1){
        if(invec->rows != 3)
            return MAT_ERR_SIZE;
        matrix_get_col(&v[0], invec, 0);
    }else{
        return MAT_ERR_SIZE;
    }
    matrix_set_row(outmat, 0, (float[]){0, -v[2], v[1]});
    matrix_set_row(outmat, 1, (float[]){v[2], 0, -v[0]});
    matrix_set_row(outmat, 2, (float[]){-v[1], v[0], 0});
    return MAT_ERR_NONE;
}

void motor_control_init(void){
    motor_wd_count = WD_DISABLE_COUNT;

    // Configure PIDs (initial state = reset and disabled)
    pid_reset(&depth_pid);
    depth_pid.kf = 0.0f;
    depth_pid.kp = 0.0f;
    depth_pid.ki = 0.0f;
    depth_pid.kd = 0.0f;
    depth_pid.min = -1.0f;
    depth_pid.max = 1.0f;

    pid_reset(&pitch_pid);
    pitch_pid.kf = 0.0f;
    pitch_pid.kp = 0.0f;
    pitch_pid.ki = 0.0f;
    pitch_pid.kd = 0.0f;
    pitch_pid.min = -1.0f;
    pitch_pid.max = 1.0f;

    pid_reset(&roll_pid);
    roll_pid.kf = 0.0f;
    roll_pid.kp = 0.0f;
    roll_pid.ki = 0.0f;
    roll_pid.kd = 0.0f;
    roll_pid.min = -1.0f;
    roll_pid.max = 1.0f;

    // Initialize all thrusters in a non-inverted state
    for(size_t i = 0; i < 8; ++i)
        motor_control_tinv[i] = false;

    // This is **NOT** the motor_matrix described in the math README
    // This is a directly constructed dof_matrix.
    // This implementation ALWAYS uses row i for motor i+1
    // row 0 = thruster 1, row 1 = thruster 2, etc
    matrix_init_static(&dof_matrix, dof_matrix_arr, 8, 6);
    //                     MotorNum-1           x       y      z     pitch    roll     yaw
    matrix_set_row(&dof_matrix, 2, (float[]){  -1,     -1,     0,      0,      0,      +1  });
    matrix_set_row(&dof_matrix, 3, (float[]){  +1,     -1,     0,      0,      0,      -1  });
    matrix_set_row(&dof_matrix, 0, (float[]){  -1,     +1,     0,      0,      0,      -1  });
    matrix_set_row(&dof_matrix, 1, (float[]){  +1,     +1,     0,      0,      0,      +1  });
    matrix_set_row(&dof_matrix, 6, (float[]){   0,      0,    -1,     -1,     -1,       0  });
    matrix_set_row(&dof_matrix, 7, (float[]){   0,      0,    -1,     -1,     +1,       0  });
    matrix_set_row(&dof_matrix, 4, (float[]){   0,      0,    -1,     +1,     -1,       0  });
    matrix_set_row(&dof_matrix, 5, (float[]){   0,      0,    -1,     +1,     +1,       0  });

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

void motor_control_raw(float s1, float s2, float s3, float s4, float s5, float s6, float s7, float s8){
    float speeds[8] = { s1, s2, s3, s4, s5, s6, s7, s8 };
    for(size_t i = 0; i < 8; ++i){
        // Apply inversions if needed
        if(motor_control_tinv[i])
            speeds[i] = -speeds[i];

        // Limit input speeds to correct range
        if(speeds[i] > 1.0)
            speeds[i] = 1.0;
        if(speeds[i] < -1.0)
            speeds[i] = -1.0;
    }
    timers_thruster_pwm_set(speeds);
}

void motor_control_local(float x, float y, float z, float pitch, float roll, float yaw){
    float target_arr[6];
    matrix target;
    matrix_init_static(&target, target_arr, 6, 1);
    matrix_set_col(&target, 0, (float[]){x, y, z, pitch, roll, yaw});

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
    // NOTE: Always call motor_control_raw NOT motor_pwm_set directly
    //       To ensure timeout works properly
    motor_control_raw(speed_arr[0], speed_arr[1], speed_arr[2], speed_arr[3],
            speed_arr[4], speed_arr[5], speed_arr[6], speed_arr[7]);
}

void motor_control_global(float x, float y, float z, float pitch, float roll, float yaw, float grav_x, float grav_y, float grav_z){
    // Construct target motion vector
    float target_arr[6];
    matrix target;
    matrix_init_static(&target, target_arr, 6, 1);
    matrix_set_col(&target, 0, (float[]){x, y, z, pitch, roll, yaw});

    // Construct gravity vector
    float gvec_arr[3];
    matrix gravity_vector;
    matrix_init_static(&gravity_vector, gvec_arr, 3, 1);
    matrix_set_col(&gravity_vector, 0, (float[]){grav_x, grav_y, grav_z});

    // b is unit gravity vector
    float b_arr[3];
    float gravl2norm;
    matrix b;
    matrix_init_static(&b, b_arr, 3, 1);
    matrix_l2vnorm(&gravl2norm, &gravity_vector);
    matrix_sc_div(&b, &gravity_vector, gravl2norm);

    // Expected unit gravity vector when "level"
    float a_arr[3];
    matrix a;
    matrix_init_static(&a, a_arr, 3, 1);
    matrix_set_col(&a, 0, (float[]){0, 0, -1});

    float v_arr[3];
    matrix v;
    matrix_init_static(&v, v_arr, 3, 1);
    matrix_vcross(&v, &a, &b);

    float c;
    matrix_vdot(&c, &a, &b);

    float sk_arr[9];
    matrix sk;
    matrix_init_static(&sk, sk_arr, 3, 3);
    skew3(&sk, &v);

    float I_arr[9];
    matrix I;
    matrix_init_static(&I, I_arr, 3, 3);
    matrix_ident(&I);

    float R_arr[9];
    matrix R;
    matrix_init_static(&R, R_arr, 3, 3);
    matrix_mul(&R, &sk, &sk);
    matrix_sc_div(&R, &R, 1 + c);
    matrix_add(&R, &R, &sk);
    matrix_add(&R, &R, &I);

    float tmp[6];
    float tltarget_arr[3], rltarget_arr[3], tgtarget_arr[3], rgtarget_arr[3];
    matrix tltarget, rltarget, tgtarget, rgtarget;
    matrix_init_static(&tltarget, tltarget_arr, 3, 1);
    matrix_init_static(&rltarget, rltarget_arr, 3, 1);
    matrix_init_static(&tgtarget, tgtarget_arr, 3, 1);
    matrix_init_static(&rgtarget, rgtarget_arr, 3, 1);
    matrix_get_col(&tmp[0], &target, 0);
    matrix_set_col(&tltarget, 0, &tmp[0]);
    matrix_set_col(&rltarget, 0, &tmp[3]);

    matrix_mul(&tgtarget, &R, &tltarget);
    matrix_mul(&rgtarget, &R, &rltarget);

    matrix_get_col(&tmp[0], &tgtarget, 0);
    matrix_get_col(&tmp[3], &rgtarget, 0);

    matrix_set_col(&target, 0, &tmp[0]);

    // Target is now a local target (stored in order in target_arr)
    motor_control_local(target_arr[0], target_arr[1], target_arr[2], target_arr[3], target_arr[4], target_arr[5]);
}

void motor_control_cfg_depth_hold(float kp, float ki, float kd, float kf, float limit){
    depth_pid.kp = kp;
    depth_pid.ki = ki;
    depth_pid.kd = kd;
    depth_pid.kf = kf;
    depth_pid.min = -limit;
    depth_pid.max = limit;
}

void motor_control_cfg_pitch_hold(float kp, float ki, float kd, float kf, float limit){
    pitch_pid.kp = kp;
    pitch_pid.ki = ki;
    pitch_pid.kd = kd;
    pitch_pid.kf = kf;
    pitch_pid.min = -limit;
    pitch_pid.max = limit;
}

void motor_control_cfg_roll_hold(float kp, float ki, float kd, float kf, float limit){
    roll_pid.kp = kp;
    roll_pid.ki = ki;
    roll_pid.kd = kd;
    roll_pid.kf = kf;
    roll_pid.min = -limit;
    roll_pid.max = limit;
}

void motor_control_sassist(float x, float y, float yaw, float pitch_target, float roll_target, float depth_target, float curr_pitch, float curr_roll, float curr_depth, float grav_x, float grav_y, float grav_z){
    // Update PID setpoints
    pitch_pid.setpoint = pitch_target;
    pitch_pid.setpoint = roll_target;
    depth_pid.setpoint = depth_target;

    // Calculate speeds using PIDs
    float z = pid_get_output(&depth_pid, curr_depth);
    float pitch = pid_get_output(&pitch_pid, curr_pitch);
    float roll = pid_get_output(&roll_pid, curr_roll);

    // Update speeds using GLOBAL mode math
    motor_control_global(x, y, z, pitch, roll, yaw, grav_x, grav_y, grav_z);
}

bool motor_control_watchdog_count(void){
    if(motor_wd_count >= WD_DISABLE_COUNT){
        // Don't increment counter further (prevents rollover issues)
        // false returned because motors were not just now disabled
        return false;
    }

    // Called every 100ms so 1 count = 100ms
    motor_wd_count++;

    // Disable after configured time
    if(motor_wd_count >= WD_DISABLE_COUNT){
        timers_thruster_pwm_set((float[]){0, 0, 0, 0, 0, 0, 0, 0});
        return true;
    }

    return false;
}

void motor_control_watchdog_feed(void){
    motor_wd_count = 0;
}

/**
 * @file motor_control.c
 * @author Marcus Behel
 */

#include <atmel_start.h>
#include <motor_control.h>
#include <motor_pwm.h>
#include <matrix.h>
#include <stdbool.h>
#include <cmdctrl.h>


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Globals
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static float dof_matrix_arr[8*6];                       // Backing array for dof matrix
static matrix dof_matrix;                               // dof matrix

static float overlap_arrs[8][8];                        // Backing arrays for overlap vectors
static matrix overlap_vectors[8];                       // overlaps vectors

static struct timer_task mc_stop_task;                  // Task to stop motors

bool motor_control_tinv[8];                             // Thruster inversion status (true = inverted)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void motor_control_cb_stop(const struct timer_task *const timer_task){
    // Watchdog timed out.
    
    // Stop all motors
    motor_pwm_set((float[]){0, 0, 0, 0, 0, 0, 0, 0});

    // Warn other end (in case it is still there and just didn't feed watchdog)
    cmdctrl_motors_killed();
}

void motor_control_init(void){
    // Initialize all thrusters in a non-inverted state
    for(size_t i = 0; i < 8; ++i)
        motor_control_tinv[i] = false;

    // This is **NOT** the motor_matrix described in the math README
    // This is a directly constructed dof_matrix.
    // This implementation ALWAYS uses row i for motor i+1
    // row 0 = thruster 1, row 1 = thruster 2, etc
    matrix_init_static(&dof_matrix, dof_matrix_arr, 8, 6);
    //                     MotorNum-1           x       y      z     pitch    roll     yaw
    matrix_set_row(&dof_matrix, 0, (float[]){  -1,     -1,     0,      0,      0,      +1  });
    matrix_set_row(&dof_matrix, 1, (float[]){  +1,     -1,     0,      0,      0,      -1  });
    matrix_set_row(&dof_matrix, 2, (float[]){  -1,     +1,     0,      0,      0,      -1  });
    matrix_set_row(&dof_matrix, 3, (float[]){  +1,     +1,     0,      0,      0,      +1  });
    matrix_set_row(&dof_matrix, 4, (float[]){   0,      0,    -1,     -1,     -1,       0  });
    matrix_set_row(&dof_matrix, 5, (float[]){   0,      0,    -1,     -1,     +1,       0  });
    matrix_set_row(&dof_matrix, 6, (float[]){   0,      0,    -1,     +1,     -1,       0  });
    matrix_set_row(&dof_matrix, 7, (float[]){   0,      0,    -1,     +1,     +1,       0  });

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

void motor_control_feed_watchdog(void){
    timer_remove_task(&TIMER_0, &mc_stop_task);     // Stop task if previously added (prevents kill)
    mc_stop_task.cb = motor_control_cb_stop;        // Configure callback
    mc_stop_task.mode = TIMER_TASK_ONE_SHOT;        // Configure mode
    mc_stop_task.interval = 1500;                   // 1.5sec before watchdog kills motors
    timer_add_task(&TIMER_0, &mc_stop_task);        // Add task now (starts 1.5sec timeout)
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
    motor_pwm_set(speeds);

    // Just updated speed.
    // Reset watchdog timeout
    motor_control_feed_watchdog();
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

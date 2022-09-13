
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <matrix.h>


int skew3(matrix *outmat, matrix *invec){
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

int main(void){
    ////////////////////////////////////////////////////////////////////////////
    /// Thruster Configuration Information
    ////////////////////////////////////////////////////////////////////////////
    // Motor Matrix Definition
    matrix motor_matrix;
    matrix_init(&motor_matrix, 8, 7);
                                              // MotorNum    x       y      z     pitch    roll     yaw
    matrix_set_row(&motor_matrix, 0, (float[]){     1,      -1,     -1,     0,      0,      0,      +1  });
    matrix_set_row(&motor_matrix, 1, (float[]){     2,      +1,     -1,     0,      0,      0,      -1  });
    matrix_set_row(&motor_matrix, 2, (float[]){     3,      -1,     +1,     0,      0,      0,      -1  });
    matrix_set_row(&motor_matrix, 3, (float[]){     4,      +1,     +1,     0,      0,      0,      +1  });
    matrix_set_row(&motor_matrix, 4, (float[]){     5,       0,      0,    -1,     -1,     -1,       0  });
    matrix_set_row(&motor_matrix, 5, (float[]){     6,       0,      0,    -1,     -1,     +1,       0  });
    matrix_set_row(&motor_matrix, 6, (float[]){     7,       0,      0,    -1,     +1,     -1,       0  });
    matrix_set_row(&motor_matrix, 7, (float[]){     8,       0,      0,    -1,     +1,     +1,       0  });

    // Construct DoF Matrix and Motor Number Vector
    matrix dof_matrix;
    float data[8];
    matrix_init(&dof_matrix, 8, 6);
    for(size_t col = 0; col < 6; ++col){
        matrix_get_col(&data[0], &motor_matrix, col + 1);
        matrix_set_col(&dof_matrix, col, &data[0]);
    }
    matrix motor_num_vec;
    matrix_init(&motor_num_vec, 8, 1);
    matrix_get_col(&data[0], &motor_matrix, 0);
    matrix_set_col(&motor_num_vec, 0, &data[0]);

    // Construct overlap vectors for each motor
    matrix contribution_matrix;
    matrix_init(&contribution_matrix, 8, 6);
    for(size_t row = 0; row < contribution_matrix.rows; ++row){
        for(size_t col = 0; col < contribution_matrix.cols; ++col){
            float dof_item;
            matrix_get_item(&dof_item, &dof_matrix, row, col);
            matrix_set_item(&contribution_matrix, row, col, (dof_item != 0) ? 1 : 0);
        }
    }

    matrix **overlap_vectors = calloc(contribution_matrix.rows, sizeof(matrix*));
    for(size_t r = 0; r < contribution_matrix.rows; ++r){
        overlap_vectors[r] = malloc(sizeof(matrix));
        matrix_init(overlap_vectors[r], contribution_matrix.rows, 1);

        // v = contribution_matrix row r transposed
        float *rowdata = calloc(contribution_matrix.rows, sizeof(float));
        matrix_get_row(rowdata, &contribution_matrix, r);
        matrix v;
        matrix_init(&v, 6, 1);
        matrix_set_col(&v, 0, rowdata);

        matrix_mul(overlap_vectors[r], &contribution_matrix, &v);

        for(size_t row = 0; row < overlap_vectors[r]->rows; ++row){
            for(size_t col = 0; col < overlap_vectors[r]->cols; ++col){
                float item;
                matrix_get_item(&item, overlap_vectors[r], row, col);
                matrix_set_item(overlap_vectors[r], row, col, (item != 0) ? 1 : 0);
            }
        }

        matrix_free(&v);
        free(rowdata);
    }

    ////////////////////////////////////////////////////////////////////////////
    /// Robot Orientation Information
    ////////////////////////////////////////////////////////////////////////////
    // Gravity vector (from accelerometer data)
    // Used to determine robot pitch and roll
    // [x, y, z] components
    matrix gravity_vector;
    matrix_init(&gravity_vector, 3, 1);                                             
    matrix_set_col(&gravity_vector, 0, (float[]){0, 1, 0});
    float grav_l2norm;
    matrix_l2vnorm(&grav_l2norm, &gravity_vector);
    matrix_sc_div(&gravity_vector, &gravity_vector, grav_l2norm);
    matrix_sc_mul(&gravity_vector, &gravity_vector, 9.81);


    ////////////////////////////////////////////////////////////////////////////
    /// Target Motion Information
    ////////////////////////////////////////////////////////////////////////////
    // Target motion in all 6 DoFs
    matrix target;
    matrix_init(&target, 6, 1);
                                        //  x       y       z     pitch    roll    yaw
    matrix_set_col(&target, 0, (float[]){   0,      1,      0,      0,      0,      0   });
    bool target_is_global = true;


    ////////////////////////////////////////////////////////////////////////////
    /// Target localization
    ////////////////////////////////////////////////////////////////////////////
    if(target_is_global){
        matrix b;
        matrix_init(&b, 3, 1);
        float gravl2norm;
        matrix_l2vnorm(&gravl2norm, &gravity_vector);
        matrix_sc_div(&b, &gravity_vector, gravl2norm);

        matrix a;
        matrix_init(&a, 3, 1);
        matrix_set_col(&a, 0, (float[]){0, 0, -1});

        matrix v;
        matrix_init(&v, 3, 1);
        matrix_vcross(&v, &a, &b);

        float c;
        matrix_vdot(&c, &a, &b);

        matrix sk;
        matrix_init(&sk, 3, 3);
        skew3(&sk, &v);

        matrix I;
        matrix_init(&I, 3, 3);
        matrix_ident(&I);

        matrix R;
        matrix_init(&R, 3, 3);
        matrix_mul(&R, &sk, &sk);
        matrix_sc_div(&R, &R, 1 + c);
        matrix_add(&R, &R, &sk);
        matrix_add(&R, &R, &I);

        matrix_free(&I);
        matrix_free(&sk);
        matrix_free(&v);
        matrix_free(&a);
        matrix_free(&b);

        float tmp[6];
        matrix tltarget, rltarget, tgtarget, rgtarget;
        matrix_init(&tltarget, 3, 1);
        matrix_init(&rltarget, 3, 1);
        matrix_init(&tgtarget, 3, 1);
        matrix_init(&rgtarget, 3, 1);
        matrix_get_col(&tmp[0], &target, 0);
        matrix_set_col(&tltarget, 0, &tmp[0]);
        matrix_set_col(&rltarget, 0, &tmp[3]);

        matrix_mul(&tgtarget, &R, &tltarget);
        matrix_mul(&rgtarget, &R, &rltarget);

        matrix_get_col(&tmp[0], &tgtarget, 0);
        matrix_get_col(&tmp[3], &rgtarget, 0);

        matrix_free(&rgtarget);
        matrix_free(&tgtarget);
        matrix_free(&rltarget);
        matrix_free(&tltarget);
        matrix_free(&R);

        matrix_set_col(&target, 0, &tmp[0]);
    }


    ////////////////////////////////////////////////////////////////////////////
    /// Motor speed calculations
    ////////////////////////////////////////////////////////////////////////////
    // In practice this would be repeated each time target changes

    // Base speed calculation
    matrix speed_vec;
    matrix_init(&speed_vec, 8, 1);
    matrix_mul(&speed_vec, &dof_matrix, &target);

    // Scale motor speeds down as needed
    while(true){
        size_t idxrow, idxcol;
        float mval;
        matrix_absmax(&mval, &idxrow, &idxcol, &speed_vec);
        if(mval <= 1)
            break;
        for(size_t i = 0; i < overlap_vectors[idxrow]->rows; ++i){
            float cval;
            matrix_get_item(&cval, overlap_vectors[idxrow], i, 0);
            if(cval == 1){
                matrix_get_item(&cval, &speed_vec, i, 0);
                cval /= mval;
                matrix_set_item(&speed_vec, i, 0, cval);
            }
        }
    }


    ////////////////////////////////////////////////////////////////////////////
    /// Print Motor Speeds
    ////////////////////////////////////////////////////////////////////////////
    for(size_t i = 0; i < motor_num_vec.rows; ++i){
        float num, speed;
        matrix_get_item(&num, &motor_num_vec, i, 0);
        matrix_get_item(&speed, &speed_vec, i, 0);
        printf("Motor %d: %4d%%\n", (int)num, (int)(speed * 100));
    }

    return EXIT_SUCCESS;
}

#include <motor_control.h>
#include <thruster.h>


bool mc_invert[8];


void mc_init(void){
    // Default all motors to non-inverted
    for(unsigned int i = 0; i < 8; ++i){
        mc_invert[i] = false;
    }
}

void mc_set_motor_matrix(matrix *motor_mat){
    // TODO: Implement actual math
}

void mc_set_raw(float *speeds){
    // Apply thruster inversions
    for(unsigned int i = 0; i < 8; ++i){
        if(mc_invert[i])
            speeds[i] *= -1;
    }

    // Actually set thruster speeds
    thruster_set(speeds);
}

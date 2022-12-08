
#include <motor_control.h>
#include <thruster.h>


// TODO: Implement motor watchdog
// TODO: Implement thruster inversions


void mc_init(void){
    // TODO: Initialize globals (once there are globals)
}

void mc_set_motor_matrix(matrix *motor_mat){
    // TODO: Implement actual math
}

void mc_set_raw(float *speeds){
    // TODO: Apply inversions
    thruster_set(speeds);
}

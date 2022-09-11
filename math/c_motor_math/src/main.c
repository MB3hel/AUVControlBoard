
#include <math.h>
#include <stdio.h>
#include <matrix.h>


int main(void){
    float dot;
    matrix cross, va, vb;
    matrix_init(&va, 1, 3);
    matrix_init(&vb, 1, 3);
    matrix_init(&cross, 1, 3);

    matrix_set_row(&va, 0, (float[]){1, 2, 3});
    matrix_set_row(&vb, 0, (float[]){2, 2, 3});

    matrix_vdot(&dot, &va, &vb);
    matrix_vcross(&cross, &va, &vb);

    printf("dot = %.2f\n\n", dot);
    matrix_print(&cross);

    return 0;
}
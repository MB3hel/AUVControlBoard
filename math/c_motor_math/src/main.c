
#include <math.h>
#include <stdio.h>
#include <matrix.h>


int main(void){
    matrix res, op1, op2;
    matrix_init(&res, 3, 2);
    matrix_init(&op1, 3, 3);
    matrix_init(&op2, 3, 2);

    matrix_set_row(&op1, 0, (float[]){1, 2, 3});
    matrix_set_row(&op1, 1, (float[]){5, 7, 4});
    matrix_set_row(&op1, 2, (float[]){1, -1, 2});

    matrix_set_row(&op2, 0, (float[]){7, 5});
    matrix_set_row(&op2, 1, (float[]){6, -5});
    matrix_set_row(&op2, 2, (float[]){4, -7});

    matrix_mul(&res, &op1, &op2);
    
    matrix_print(&op1);
    printf("\n");
    matrix_print(&op2);
    printf("\n");
    matrix_print(&res);

    return 0;
}
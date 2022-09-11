
#include <math.h>
#include <stdio.h>
#include <matrix.h>


int main(void){
    matrix mat;
    matrix_init(&mat, 4, 4);

    matrix_set_row(&mat, 0, (float[]){5, 7, 9, 1});
    matrix_set_row(&mat, 1, (float[]){0, 1, 0, 0});
    matrix_set_row(&mat, 2, (float[]){0, 0, 5, 0});
    matrix_set_row(&mat, 3, (float[]){0, 0, 0, 6});
    
    matrix_print(&mat);
    printf("\n");

    matrix mat_inv;;
    matrix_init(&mat_inv, 4, 4);

    matrix_inv(&mat_inv, &mat);
    matrix_print(&mat_inv);

    return 0;
}

#include <math.h>
#include <stdio.h>
#include <matrix.h>


int main(void){
    matrix mat;

    matrix_init(&mat, 8, 8);
    matrix_set_row(&mat, 4, (float[]){0.5, 2, 3, 4, 5, 6, 7, 8});
    matrix_set_col(&mat, 7, (float[]){4, 5, 6, 7, 8, 9, 10, 11});
    matrix_print(&mat);

    printf("\n");

    float data[8];
    matrix_get_row(&mat, 4, (float*)&data);
    for(size_t i = 0; i < 8; ++i){
        printf("%.2f\t", data[i]);
    }
    printf("\n");

    matrix_get_col(&mat, 7, (float*)&data);
    for(size_t i = 0; i < 8; ++i){
        printf("%.2f\t", data[i]);
    }
    printf("\n");
}
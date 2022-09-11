
#include <matrix.h>
#include <stdlib.h>
#include <stdio.h>


////////////////////////////////////////////////////////////////////////////////
/// Macos
////////////////////////////////////////////////////////////////////////////////

#define MATRIX_IDX(m, row, col)     (m->cols * row + col)


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////

void matrix_init(matrix *m, size_t rows, size_t cols){
    m->data = calloc(rows * cols, sizeof(float));
    m->rows = rows;
    m->cols = cols;
}

void matrix_free(matrix *m){
    free(m->data);
}

void matrix_print(matrix *m){
    for(size_t row = 0; row < m->rows; ++row){
        for(size_t col = 0; col < m->cols; ++col){
            printf("%.2f\t", m->data[MATRIX_IDX(m, row, col)]);
        }
        printf("\n");
    }
}

void matrix_set_item(matrix *m, size_t row, size_t col, float item){
    m->data[MATRIX_IDX(m, row, col)] = item;
}

void matrix_set_row(matrix *m, size_t row, float *data){
    for(size_t col = 0; col < m->cols; ++col){
        m->data[MATRIX_IDX(m, row, col)] = data[col];
    }
}

void matrix_set_col(matrix *m, size_t col, float *data){
    for(size_t row = 0; row < m->rows; ++row){
        m->data[MATRIX_IDX(m, row, col)] = data[row];
    }
}

float matrix_get_item(matrix *m, size_t row, size_t col){
    return m->data[MATRIX_IDX(m, row, col)];
}

void matrix_get_row(matrix *m, size_t row, float *data){
    for(size_t col = 0; col < m->cols; ++col){
        data[col] = m->data[MATRIX_IDX(m, row, col)];
    }
}

void matrix_get_col(matrix *m, size_t col, float *data){
    for(size_t row = 0; row < m->rows; ++row){
        data[row] = m->data[MATRIX_IDX(m, row, col)];
    }
}

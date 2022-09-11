
#include <matrix.h>
#include <stdlib.h>


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

void matrix_set_item(matrix *m, size_t row, size_t col, float item){
    m->data[MATRIX_IDX(m, row, col)] = item;
}

float matrix_get_item(matrix *m, size_t row, size_t col){
    return m->data[MATRIX_IDX(m, row, col)];
}

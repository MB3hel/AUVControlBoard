
#include <matrix.h>
#include <stdlib.h>
#include <stdio.h>


////////////////////////////////////////////////////////////////////////////////
/// Macos
////////////////////////////////////////////////////////////////////////////////

#define MAT_IDX(m, row, col)                (m->cols * row + col)


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////

int matrix_init(matrix *m, size_t rows, size_t cols){
    if(rows == 0 || cols == 0)
        return MAT_ERR_SIZE;
    m->data = calloc(rows * cols, sizeof(float));
    m->rows = rows;
    m->cols = cols;
    return MAT_ERR_NONE;
}

void matrix_free(matrix *m){
    free(m->data);
}

void matrix_print(matrix *m){
    for(size_t row = 0; row < m->rows; ++row){
        for(size_t col = 0; col < m->cols; ++col){
            printf("%.2f\t", m->data[MAT_IDX(m, row, col)]);
        }
        printf("\n");
    }
}

int matrix_set_item(matrix *m, size_t row, size_t col, float item){
    if(row >= m->rows || col >= m->cols)
        return MAT_ERR_INDX;
    m->data[MAT_IDX(m, row, col)] = item;
    return MAT_ERR_NONE;
}

int matrix_set_row(matrix *m, size_t row, float *data){
    if(row >= m->rows)
        return MAT_ERR_INDX;
    for(size_t col = 0; col < m->cols; ++col){
        m->data[MAT_IDX(m, row, col)] = data[col];
    }
    return MAT_ERR_NONE;
}

int matrix_set_col(matrix *m, size_t col, float *data){
    if(col >= m->cols)
        return MAT_ERR_INDX;
    for(size_t row = 0; row < m->rows; ++row){
        m->data[MAT_IDX(m, row, col)] = data[row];
    }
    return MAT_ERR_NONE;
}

int matrix_get_item(matrix *m, size_t row, size_t col, float *item){
    if(row >= m->rows || col >= m->cols)
        return MAT_ERR_INDX;
    *item = m->data[MAT_IDX(m, row, col)];
    return MAT_ERR_NONE;
}

int matrix_get_row(matrix *m, size_t row, float *data){
    if(row >= m->rows)
        return MAT_ERR_INDX;
    for(size_t col = 0; col < m->cols; ++col){
        data[col] = m->data[MAT_IDX(m, row, col)];
    }
    return MAT_ERR_NONE;
}

int matrix_get_col(matrix *m, size_t col, float *data){
    if(col >= m->cols)
        return MAT_ERR_INDX;
    for(size_t row = 0; row < m->rows; ++row){
        data[row] = m->data[MAT_IDX(m, row, col)];
    }
    return MAT_ERR_NONE;
}

int matrix_add(matrix *dest, matrix *src1, matrix *src2){
    if(dest->rows != src1->rows || dest->rows != src2->cols)
        return MAT_ERR_SIZE;
    for(size_t row = 0; row < dest->rows; ++row){
        for(size_t col = 0; col < dest->cols; ++col){
            dest->data[MAT_IDX(dest, row, col)] = src1->data[MAT_IDX(src1, row, col)] + src1->data[MAT_IDX(src2, row, col)];
        }
    }
    return MAT_ERR_NONE;
}

int matrix_sub(matrix *dest, matrix *src1, matrix *src2){
    if(dest->rows != src1->rows || dest->rows != src2->cols)
        return MAT_ERR_SIZE;
    for(size_t row = 0; row < dest->rows; ++row){
        for(size_t col = 0; col < dest->cols; ++col){
            dest->data[MAT_IDX(dest, row, col)] = src1->data[MAT_IDX(src1, row, col)] - src1->data[MAT_IDX(src2, row, col)];
        }
    }
    return MAT_ERR_NONE;
}

int matrix_ew_mul(matrix *dest, matrix *src1, matrix *src2){
    if(dest->rows != src1->rows || dest->rows != src2->cols)
        return MAT_ERR_SIZE;
    for(size_t row = 0; row < dest->rows; ++row){
        for(size_t col = 0; col < dest->cols; ++col){
            dest->data[MAT_IDX(dest, row, col)] = src1->data[MAT_IDX(src1, row, col)] * src1->data[MAT_IDX(src2, row, col)];
        }
    }
    return MAT_ERR_NONE;
}

int matrix_ew_div(matrix *dest, matrix *src1, matrix *src2){
    if(dest->rows != src1->rows || dest->rows != src2->cols)
        return MAT_ERR_SIZE;
    for(size_t row = 0; row < dest->rows; ++row){
        for(size_t col = 0; col < dest->cols; ++col){
            dest->data[MAT_IDX(dest, row, col)] = src1->data[MAT_IDX(src1, row, col)] / src1->data[MAT_IDX(src2, row, col)];
        }
    }
    return MAT_ERR_NONE;
}


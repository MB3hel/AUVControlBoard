#pragma once

#include <stddef.h>


////////////////////////////////////////////////////////////////////////////////
/// Typedefs
////////////////////////////////////////////////////////////////////////////////

typedef struct{
    float *data;
    size_t rows;
    size_t cols;
} matrix;


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////

/**
 * Initialize float matrix
 * @param m Pointer to the matrix to initialize
 * @param rows Number of rows in matrix
 * @param cols Number of columns in matrix
 */
void matrix_init(matrix *m, size_t rows, size_t cols);

/**
 * Free float matrix
 * @param m Pointer to the matrix to free
 */
void matrix_free(matrix *m);

/**
 * Set an item in a matrix
 * @param m Pointer to the matrix to set an item in
 * @param row Row to set item in
 * @param col Column to set item in
 * @param item Value to set
 */
void matrix_set_item(matrix *m, size_t row, size_t col, float item);

/**
 * Get an item in a matrix
 * @param m Pointer to the matrix to set an item in
 * @param row Row to set item in
 * @param col Column to set item in
 * @return Value of the item at the given position
 */
float matrix_get_item(matrix *m, size_t row, size_t col);

// TODO: Set rows and columns

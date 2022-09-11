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
 * Print the given matrix
 * @param m Pointer to the matrix to print
 */
void matrix_print(matrix *m);

/**
 * Set an item in a matrix
 * @param m Pointer to the matrix to set an item in
 * @param row Row to set item in
 * @param col Column to set item in
 * @param item Value to set
 */
void matrix_set_item(matrix *m, size_t row, size_t col, float item);

/**
 * Set a row in a matrix
 * @param m Pointer to the matrix to set a row in
 * @param row Row to set
 * @param data Pointer to data to insert into row (must be same size as the row)
 */
void matrix_set_row(matrix *m, size_t row, float *data);

/**
 * Set a column in a matrix
 * @param m Pointer to the matrix to set a column in
 * @param col Column to set
 * @param data Pointer to data to insert into column (must be same size as the column)
 */
void matrix_set_col(matrix *m, size_t col, float *data);

/**
 * Get an item in a matrix
 * @param m Pointer to the matrix to set an item in
 * @param row Row to set item in
 * @param col Column to set item in
 * @return Value of the item at the given position
 */
float matrix_get_item(matrix *m, size_t row, size_t col);

/**
 * Get a row in a matrix
 * @param m Pointer to the matrix to get a row in
 * @param row Row to get
 * @param data Pointer to location to read matrix row into (must be same size as a row)
 */
void matrix_get_row(matrix *m, size_t row, float *data);

/**
 * Get a column in a matrix
 * @param m Pointer to the matrix to get a column in
 * @param col Column to get
 * @param data Pointer to location to read matrix column into (must be same size as a column)
 */
void matrix_get_col(matrix *m, size_t col, float *data);

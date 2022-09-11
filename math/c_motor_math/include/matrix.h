#pragma once

#include <stddef.h>


////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////

#define MAT_ERR_NONE                0   // No error
#define MAT_ERR_SIZE                -1  // Invalid size(s)
#define MAT_ERR_INDX                -2  // Invalid index


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
 * @return Error code
 */
int matrix_init(matrix *m, size_t rows, size_t cols);

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
 * @return Error code
 */
int matrix_set_item(matrix *m, size_t row, size_t col, float item);

/**
 * Set a row in a matrix
 * @param m Pointer to the matrix to set a row in
 * @param row Row to set
 * @param data Pointer to data to insert into row (must be same size as the row)
 * @return Error code
 */
int matrix_set_row(matrix *m, size_t row, float *data);

/**
 * Set a column in a matrix
 * @param m Pointer to the matrix to set a column in
 * @param col Column to set
 * @param data Pointer to data to insert into column (must be same size as the column)
 * @return Error code
 */
int matrix_set_col(matrix *m, size_t col, float *data);

/**
 * Get an item in a matrix
 * @param m Pointer to the matrix to set an item in
 * @param row Row to set item in
 * @param col Column to set item in
 * @param item Pointer to float to store item in
 * @return Error code
 */
int matrix_get_item(matrix *m, size_t row, size_t col, float *item);

/**
 * Get a row in a matrix
 * @param m Pointer to the matrix to get a row in
 * @param row Row to get
 * @param data Pointer to location to read matrix row into (must be same size as a row)
 * @return Error code
 */
int matrix_get_row(matrix *m, size_t row, float *data);

/**
 * Get a column in a matrix
 * @param m Pointer to the matrix to get a column in
 * @param col Column to get
 * @param data Pointer to location to read matrix column into (must be same size as a column)
 * @return Error code
 */
int matrix_get_col(matrix *m, size_t col, float *data);

/**
 * Add two matrices and store the result in a third matrix
 * dest = src1 + src2
 * All three matrices must be the same size
 * @param dest Pointer to destination matrix
 * @param src1 Pointer to input operand 1
 * @param src2 Pointer to input operand 2
 * @return Error code
 */
int matrix_add(matrix *dest, matrix *src1, matrix *src2);

/**
 * Add two matrices and store the result in a third matrix
 * dest = src1 + src2
 * All three matrices must be the same size
 * @param dest Pointer to destination matrix
 * @param src1 Pointer to input operand 1
 * @param src2 Pointer to input operand 2
 * @return Error code
 */
int matrix_add(matrix *dest, matrix *src1, matrix *src2);

/**
 * Subtract two matrices and store the result in a third matrix
 * dest = src1 - src2
 * All three matrices must be the same size
 * @param dest Pointer to destination matrix
 * @param src1 Pointer to input operand 1
 * @param src2 Pointer to input operand 2
 * @return Error code
 */
int matrix_sub(matrix *dest, matrix *src1, matrix *src2);

/**
 * Multiply corresponding elements of the two source matrices and store the
 * result in the destination matrix.
 * @param dest Pointer to destination matrix
 * @param src1 Pointer to input operand 1
 * @param src2 Pointer to input operand 2
 * @return Error code
 */
int matrix_ew_mul(matrix *dest, matrix *src1, matrix *src2);

/**
 * Divide corresponding elements of the two source matrices and store the
 * result in the destination matrix.
 * @param dest Pointer to destination matrix
 * @param src1 Pointer to input operand 1
 * @param src2 Pointer to input operand 2
 * @return Error code
 */
int matrix_ew_div(matrix *dest, matrix *src1, matrix *src2);

// TODO: Elementwise multiply and divide

// TODO: Transpose

// TODO: Matrix multiply

// TODO: Inverse

// TODO: Vector dot

// TODO: Vector cross

// TODO: Scalar multiply and divide

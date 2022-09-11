#pragma once

#include <stddef.h>


////////////////////////////////////////////////////////////////////////////////
/// Macros
////////////////////////////////////////////////////////////////////////////////

// Settings
#define MAT_EN_PRINT                    // Uncomment to enable matrix_print func

// Error Codes
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
 * Copy one matrix into another
 * @param dest Pointer to destination matrix
 * @param src Pointer to source matrix
 * @return Error code
 */
int matrix_copy(matrix *dest, matrix *src);

/**
 * Fill matrix with zeros
 * @param m Pointer to matrix to zero
 * @return Error code
 */
int matrix_zeros(matrix *m);

/**
 * Fill matrix with ones
 * @param m Pointer to matrix to zero
 * @return Error code
 */
int matrix_ones(matrix *m);

/**
 * Make into identity matrix
 * @param m Pointer to matrix to zero
 * @return Error code
 */
int matrix_ident(matrix *m);

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
 * @param item Pointer to float to store item in
 * @param m Pointer to the matrix to set an item in
 * @param row Row to set item in
 * @param col Column to set item in
 * @return Error code
 */
int matrix_get_item(float *item, matrix *m, size_t row, size_t col);

/**
 * Get a row in a matrix
 * @param data Pointer to location to read matrix row into (must be same size as a row)
 * @param m Pointer to the matrix to get a row in
 * @param row Row to get
 * @return Error code
 */
int matrix_get_row(float *data, matrix *m, size_t row);

/**
 * Get a column in a matrix
 * @param data Pointer to location to read matrix column into (must be same size as a column)
 * @param m Pointer to the matrix to get a column in
 * @param col Column to get
 * @return Error code
 */
int matrix_get_col(float *data, matrix *m, size_t col);

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

/**
 * Multiply a matrix by a scalar
 * @param dest Pointer to destination matrix
 * @param src Pointer to source matrix
 * @param scalar Scalar to multiply matrix by
 * @return Error code
 */
int matrix_sc_mul(matrix *dest, matrix *src, float scalar);

/**
 * Divide a matrix by a scalar
 * @param dest Pointer to destination matrix
 * @param src Pointer to source matrix
 * @param scalar Scalar to divide matrix by
 * @return Error code
 */
int matrix_sc_div(matrix *dest, matrix *src, float scalar);

/**
 * Multiply two matrices and store the result in another
 * dest = op1 * op2
 * @param dest Pointer to destination matrix
 * @param op1 Pointer to operand 1
 * @param op2 Pointer to operand 2
 * @return Error code
 */
int matrix_mul(matrix *dest, matrix *op1, matrix *op2);

/**
 * Transpose the input matrix and store it in the output matrix
 * @param dest Pointer to destination matrix
 * @param src Pointer to input matrix
 * @return Error code
 */
int matrix_transpose(matrix *dest, matrix *src);

/**
 * Calculate the determinant of a matrix
 * @param det Pointer to location to store determinant
 * @param m Matrix to calculate determinant of
 * @return Error code
 */
int matrix_det(float *det, matrix *m);

/**
 * Calculate a cofactor matrix
 * @param dest Destination to store cofactor matrix
 * @param m Matrix to calculate cofactor matrix of
 * @return Error code
 */
int matrix_cofactor(matrix *dest, matrix *m);

/**
 * Calculate an inverse matrix
 * @param dest Destination to store inverse matrix
 * @param src Matrix to calculate inverse of
 * @return Error code
 */
int matrix_inv(matrix *dest, matrix *src);

// TODO: Vector dot

// TODO: Vector cross


#include <matrix.h>
#include <stdlib.h>


////////////////////////////////////////////////////////////////////////////////
/// Functions
////////////////////////////////////////////////////////////////////////////////

void matrix_init(matrix *m, size_t dim0, size_t dim1){
    m->data = calloc(dim0 * dim1, sizeof(float));
    m->dim[0] = dim0;
    m->dim[1] = dim1;
}

void matrix_free(matrix *m){
    free(m->data);
}

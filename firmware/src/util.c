
#include <util.h>

static unsigned int rand_seed = 5323;

void srand(unsigned int seed){
    rand_seed = seed;
}

unsigned int rand(){
    rand_seed = (8253729 * rand_seed + 2396403); 
    return rand_seed  % 32767;
}
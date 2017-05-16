#ifndef __CPPLIB_BOARD_H
#define __CPPLIB_BOARD_H

#if defined(BOARD_PCA10056)
#include "boards//cl_board_pca10056.h"
#elif defined(BOARD_PCA10040)
#include "boards//cl_board_pca10040.h"
#else 
#error Unknown board, or board not defined!
#endif

#endif

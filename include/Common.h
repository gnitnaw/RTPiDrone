/*!
 * \file    Common.h
 * \brief   Some functions frequently used by the defined objects.
 */
#ifndef H_COMMON
#define H_COMMON
#include <stdint.h>
void exchange(char*, int);
void _usleep(int);
float getSqrt(float*, int);
uint64_t get_nsec(void);
#endif

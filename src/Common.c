/*!
 * \file    Common.c
 * \brief   Realization of the function defined in Common.h
 */
#include "Common.h"
#include <unistd.h>
#include <time.h>
#include <math.h>

/*!
 * \fn      void exchange(char* buf, int len)
 * \brief   Function to switch the endianness for 2-byte data (swap ith and (i+1)th, i = even)
 * \param buf Pointer where the data is
 * \param len How many (int16_t) data which the buf contains.
 */
void exchange(char* buf, int len)
{
    char tmp;
    int i;
    for (i=0; i<len; ++i) {
        tmp = buf[i];
        buf[i] = buf[i+1];
        buf[i+1] = tmp;
        ++i;
    }
}

/*!
 * \fn      void _usleep(int micro)
 * \brief   High-resolution sleep (with micro-second unit)
 * \param micro Duration of sleep (micro-second)
 */
void _usleep(int micro)
{
    struct timespec req = {0};
    req.tv_sec = 0;
    req.tv_nsec = micro * 1000L;
    nanosleep(&req, (struct timespec *)NULL);
}

/*!
 * \fn      float getSqrt(float* v, int N)
 * \brief   Get the square-root of the square-sum : sqrt(v0*v0 + v1*v1 + ... + vN-1*vN-1)
 * \param v Pointer of the data
 * \param N How many elements which the data has.
 */
float getSqrt(float* v, int N)
{
    float sum = 0.0f;
    for (int i=0; i<N; ++i) {
        sum += pow(v[i],2);
    }
    return sqrtf(sum);
}

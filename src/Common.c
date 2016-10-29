#include "Common.h"
#include <unistd.h>
#include <time.h>
#include <math.h>

/*!
 * \fn      void exchange(char* buf, int len);
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

void _usleep(int micro)
{
    struct timespec req = {0};
    req.tv_sec = 0;
    req.tv_nsec = micro * 1000L;
    nanosleep(&req, (struct timespec *)NULL);
}

float getSqrt(float* v, int N)
{
    float sum = 0.0f;
    for (int i=0; i<N; ++i) {
        sum += pow(v[i],2);
    }
    return sqrtf(sum);
}

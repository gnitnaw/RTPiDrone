/* The original file is from https://github.com/tmrh20/RF24/ .
 * In this repository, it has been modified by Wan-Ting CHEN (wanting@gmail.com)
 * in order to be compatible with latest version of bcm2835 library.
 */

/*
   2  Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>
   3
   4  This program is free software; you can redistribute it and/or
   5  modify it under the terms of the GNU General Public License
   6  version 2 as published by the Free Software Foundation.
   7  */

#include "RF24/compatibility.h"


static long mtime, seconds, useconds;
static struct timeval start, end;

/**********************************************************************/
/**
 * This function is added in order to simulate arduino delay() function
 * @param milisec
 */
void __msleep(int milisec)
{
    struct timespec req = {0};
    req.tv_sec = 0;
    req.tv_nsec = milisec * 1000000L;
    nanosleep(&req, (struct timespec *)NULL);
}

void __usleep(int milisec)
{
    struct timespec req = {0};
    req.tv_sec = 0;
    req.tv_nsec = milisec * 1000L;
    nanosleep(&req, (struct timespec *)NULL);
}

/**
 * This function is added in order to simulate arduino millis() function
 */


void __start_timer(void)
{
    gettimeofday(&start, NULL);
}

long __millis()
{
    gettimeofday(&end, NULL);
    seconds  = end.tv_sec  - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;

    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
    return mtime;
}

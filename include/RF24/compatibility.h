/* The original file is from https://github.com/tmrh20/RF24/ .
 * In this repository, it has been modified by Wan-Ting CHEN (wanting@gmail.com)
 * in order to be compatible with latest version of bcm2835 library.
 */
 
/*
 * File:   compatiblity.h
 * Author: purinda
 *
 * Created on 24 June 2012, 3:08 PM
 */

#ifndef COMPATIBLITY_H
#define	COMPATIBLITY_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <time.h>
#include <sys/time.h>

void __msleep(int milisec);
void __usleep(int milisec);
void __start_timer(void);
long __millis();
inline long millis()
{
    return __millis();
}
#ifdef	__cplusplus
}
#endif

//long millis(){return __millis();};
#endif	/* COMPATIBLITY_H */


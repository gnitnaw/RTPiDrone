/*
    Quadcopter -- RF24_Interface.h
    Auther: Wan-Ting CHEN (wanting@gmail.com)
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef H_RF24_INTERFACE
#define H_RF24_INTERFACE
#include <stddef.h>
#ifdef __cplusplus
extern "C" {
#endif

void RF24WT_init(void);
void RF24WT_exchangeInfo(unsigned char *in, unsigned char *out);
void RF24WT_exchangeInfo_Count(unsigned long *in, unsigned long *out, int *nTime);
int RF24WT_receiveInfo(unsigned char *in, size_t ssize);
void RF24WT_transmitInfo(unsigned char *out, size_t ssize);

#ifdef __cplusplus
}
#endif
#endif

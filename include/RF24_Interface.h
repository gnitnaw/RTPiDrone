#ifndef H_RF24_INTERFACE
#define H_RF24_INTERFACE

#ifdef __cplusplus
extern "C" {
#endif

void RF24WT_init(void);
void RF24WT_exchangeInfo(unsigned char *in, unsigned char *out);
void RF24WT_exchangeInfo_Count(unsigned long *in, unsigned long *out, int *nTime);
int RF24WT_receiveInfo(unsigned char *in, int ssize);
void RF24WT_transmitInfo(unsigned char *out, int ssize);

#ifdef __cplusplus
}
#endif
#endif

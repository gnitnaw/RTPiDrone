#ifndef H_DRONE_COMMAND
#define H_DRONE_COMMAND
#include <stdint.h>

typedef struct {
    uint8_t control[4];
    signed char horDirection[2];
    signed char verDirection;
    signed char rotateDirection;
    float angle_expect[3];
    unsigned int power;
    unsigned char switchValue;
    unsigned int zeroCount;
} Drone_Command;

void Drone_Command_Decode(Drone_Command*, const unsigned char*);
#endif

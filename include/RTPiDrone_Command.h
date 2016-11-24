#ifndef H_DRONE_COMMAND
#define H_DRONE_COMMAND

typedef struct {
    signed char horDirection[2];
    signed char verDirection;
    signed char rotateDirection;
    float angle_expect[3];
    unsigned int power;
    unsigned char switchValue;
} Drone_Command;

void Drone_Command_Decode(Drone_Command*, const unsigned char*);
#endif

#ifndef H_DRONE_PID
#define H_DRONE_PID

typedef struct Drone_PID    Drone_PID;

void Drone_PID_Init(Drone_PID**);
void Drone_PID_Delete(Drone_PID**);
void Drone_PID_update(Drone_PID*, float*, float*, float*, int*, float*, unsigned int*);

#endif

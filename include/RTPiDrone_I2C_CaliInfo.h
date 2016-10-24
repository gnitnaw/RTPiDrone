#ifndef H_DRONE_I2C_CALIINFO
#define H_DRONE_I2C_CALIINFO

typedef struct Drone_I2C_CaliInfo  Drone_I2C_CaliInfo;

void Drone_I2C_Cali_Init(Drone_I2C_CaliInfo**, int);//!< \private \memberof Drone_I2C_CaliInfo: Initialize Drone_I2C_CaliInfo
void Drone_I2C_Cali_Delete(Drone_I2C_CaliInfo**);   //!< \private \memberof Drone_I2C_CaliInfo: Terminate Drone_I2C_CaliInfo
float* Drone_I2C_Cali_getMean(Drone_I2C_CaliInfo*); //!< \private \memberof Drone_I2C_CaliInfo: Get mean values from calibration
float* Drone_I2C_Cali_getSD(Drone_I2C_CaliInfo*);   //!< \private \memberof Drone_I2C_CaliInfo: Get standard deviation
#endif

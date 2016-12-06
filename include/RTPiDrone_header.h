#ifndef H_DRONE_HEADER
#define H_DRONE_HEADER

#define DEBUG
//#define HMC5883L_PWM_CALI
#define CONTROL_PERIOD      (4000000L)
#define KP                  (7.0f)
#define KI                  (0.2f)
#define KD                  (105.0f)
#define PWM_MAX             (3280)
#define PWM_MIN             (1640)
#define ADXL345_RATE        (400)
#define L3G4200D_RATE       (400)
//#define HMC5883L_PERIOD     (6000000L)
#define HMC5883L_RATE       (75)
#define HMC5883L_PERIOD     (1000000000L/HMC5883L_RATE)
#define BMP085_PeriodLong   (25500000L)
#define BMP085_PeriodShort  (4500000L)
#endif

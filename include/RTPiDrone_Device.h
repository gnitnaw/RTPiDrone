#ifndef  H_DRONE_DEVICE
#define  H_DRONE_DEVICE
#include <stdint.h>

/*!
 * Drone_Device type.
 * Prototype of all Devices.
 */
typedef struct {
    char name[16];                  //!< \private Name of device.
    int (*init_func)(void*);        //!< \private Initialization function
    int (*rawdata_func)(void*);     //!< \private Function to get raw data from device
    int (*data_func)(void*);        //!< \private Function to convert raw data to real data
    int (*end_func)(void*);         //!< \private Termination of device
    void*	getData;	            //!< \private Pointer of real data
    uint64_t    lastUpdate;         //!< \private Last update time
    uint64_t    period;             //!< \private Period for sensor refresh
} Drone_Device;

/*!
 * Set name of device.
 * \public \memberof Drone_Device
 */
void Drone_Device_SetName(Drone_Device*, const char*);

/*!
 * Set initialization function.
 * \public \memberof Drone_Device
 */
void Drone_Device_SetInitFunction(Drone_Device*, int (*)(void*));

/*!
 * Set function to get raw data from device
 * \public \memberof Drone_Device
 */
void Drone_Device_SetRawFunction(Drone_Device*, int (*)(void*));

/*!
 * Set function to convert raw data to real data.
 * \public \memberof Drone_Device
 */
void Drone_Device_SetRealFunction(Drone_Device*, int (*)(void*));

/*!
 * Set termination of device
 * \public \memberof Drone_Device
 */
void Drone_Device_SetEndFunction(Drone_Device*, int (*)(void*));

/*!
 * Set data point of device
 * \public \memberof Drone_Device
 */
void Drone_Device_SetDataPointer(Drone_Device*, void*);

/*!
 * Create an device
 * \public \memberof Drone_Device
 */
void Drone_Device_Create(Drone_Device*);

/*!
 * Initialize an device
 * \public \memberof Drone_Device
 */
int Drone_Device_Init(Drone_Device*);

/*!
 * Get raw data
 * \public \memberof Drone_Device
 */
int Drone_Device_GetRawData(Drone_Device*);

/*!
 * Convert raw to real data
 * \public \memberof Drone_Device
 */
int Drone_Device_GetRealData(Drone_Device*);

/*!
 * Terminate an device
 * \public \memberof Drone_Device
 */
int Drone_Device_End(Drone_Device*);

/*!
 * return the location of real data
 * \public \memberof Drone_Device
 */
void* Drone_Device_GetData(Drone_Device*);

/*!
 * return the name of device
 * \public \memberof Drone_Device
 */
char* Drone_Device_GetName(Drone_Device*);

/*!
 * return the location of real data after refreshing.
 * \public \memberof Drone_Device
 */
void* Drone_Device_GetRefreshedData(Drone_Device*, uint64_t*);

void Drone_Device_SetPeriod(Drone_Device* dev, uint64_t);
#endif

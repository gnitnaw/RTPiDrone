#ifndef H_RTPIDRONE_SPI_DEVICE_MCP3008
#define H_RTPIDRONE_SPI_DEVICE_MCP3008
/*!
 * Drone_SPI_Device_MCP3008 class.
 * \extends Drone_SPI_Device
 */
typedef struct Drone_SPI_Device_MCP3008 Drone_SPI_Device_MCP3008;   //!< Drone_SPI_Device_MCP3008 type
/*!
 * Setup of MCP3008.
 * \public \memberof Drone_SPI_Device_MCP3008
 */
int MCP3008_setup(Drone_SPI_Device_MCP3008**);

/*!
 * Remove of MCP3008.
 * \public \memberof Drone_SPI_Device_MCP3008
 */
void MCP3008_delete(Drone_SPI_Device_MCP3008**);

#endif


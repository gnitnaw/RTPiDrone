# The project version number.
set(VERSION_MAJOR   0   CACHE STRING "Project major version number.")
set(VERSION_MINOR   0   CACHE STRING "Project minor version number.")
set(VERSION_PATCH   1   CACHE STRING "Project patch version number.")
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)
set(I2C_ELEMENT
  RTPiDrone_I2C_Device_ADXL345.c
  RTPiDrone_I2C_Device_L3G4200D.c
  RTPiDrone_I2C_Device_HMC5883L.c
  RTPiDrone_I2C_Device_BMP085.c
  RTPiDrone_I2C_Device_PCA9685PW.c
)
set(SPI_ELEMENT
    RTPiDrone_SPI_Device_MCP3008.c
    RTPiDrone_SPI_Device_RF24.c
)
set(MAIN_ELEMENT
    RTPiDrone_I2C.c
    RTPiDrone_Device.c
    RTPiDrone_SPI.c
    RTPiDrone.c
)
set(RF24_ELEMENT
    RF24/RF24.cpp 
    RF24/spi.cpp 
    RF24/compatibility.c 
    RF24/RF24_Interface.cpp    
)
include_directories(${RTPiDrone_SOURCE_DIR}/include)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic -O3 -std=gnu11")
link_directories(${RTPiDrone_SOURCE_DIR}/src)
add_library(RF24WT ${RF24_ELEMENT})
add_executable(RTPiDrone ${MAIN_ELEMENT} ${I2C_ELEMENT} ${SPI_ELEMENT} Common.c main.c)
target_link_libraries(RTPiDrone RF24WT -lbcm2835 -lgsl -lgslcblas -lpthread -lm -lrt)

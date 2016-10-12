# The project version number.
set(VERSION_MAJOR   0   CACHE STRING "Project major version number.")
set(VERSION_MINOR   0   CACHE STRING "Project minor version number.")
set(VERSION_PATCH   1   CACHE STRING "Project patch version number.")
set(I2C_ELEMENT
  RTPiDrone_I2C_Device_ADXL345.c
  RTPiDrone_I2C_Device_L3G4200D.c
  RTPiDrone_I2C_Device_DOF6.c
  RTPiDrone_I2C_Device_HMC5883L.c
)
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)
include_directories(${RTPiDrone_SOURCE_DIR}/include)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic -O3 -std=gnu11")
add_executable(RTPiDrone RTPiDrone_I2C_Device.c ${I2C_ELEMENT} RTPiDrone_I2C.c RTPiDrone.c main.c)
target_link_libraries(RTPiDrone -lbcm2835)

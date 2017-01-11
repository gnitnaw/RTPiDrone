# RTPiDrone
Real-Time Raspberry Pi Drone (RTPiDrone)

### Requirement ###

#### Operating System ###

Linux Preempt_RT or Xenomai

#### Hardware ####

- GY80 (or ST Microelectronics L3G4200D + Analog Devices ADXL345 + Honeywell MC5883L + Bosch BMP085)
  http://selfbuilt.net/shop/gy-80-inertial-management-unit

- MS5611 (optional, to have better precision of attitude)
  http://www.te.com/usa-en/product-CAT-BLPS0036.html
  
- PCA9685PW (to adjust the power of motor)
  https://www.adafruit.com/product/815

- MCP3008 (to get the voltage value of battery(single cell) )
  https://www.adafruit.com/product/856

- nRF24L01+ * 2 (for the communication between the controller and drone)
  https://www.nordicsemi.com/eng/Products/2.4GHz-RF/nRF24L01P

- Arduino (to be the controller)

- Raspberry pi (RPi) model B+ or RPi2/3 model B

#### Software ####

- bcm2835 : C library for Broadcom BCM 2835 as used in Raspberry Pi 
  http://www.airspayce.com/mikem/bcm2835/

- GSL - GNU Scientific Library
  https://www.gnu.org/software/gsl/

- cmake

- Doxygen

#### Installation ####
- Download the source code
- cd RTPiDrone
- mkdir build
- cd build
- cmake ..
- make doc (optional, for generating document)
- make (make sure you already install necessary libraries)
- The excutable file will be in ./src/RTPiDrone

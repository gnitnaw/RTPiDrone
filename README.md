# RTPiDrone
Real-Time Raspberry Pi Drone (RTPiDrone)

### Requirement ###

#### Operating System ###

Linux Preempt_RT or Xenomai

#### Hardware ####

- GY80 (or ST Microelectronics L3G4200D + Analog Devices ADXL345 + Honeywell MC5883L + Bosch BMP085)
  http://selfbuilt.net/shop/gy-80-inertial-management-unit

- PCA9685PW
  https://www.adafruit.com/product/815

- MCP3008
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

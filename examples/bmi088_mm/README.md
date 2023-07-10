# BMI088_MM examples

## Examples overview

### `common`
- For the sake of simplicity of example code, repeated code is placed in `common.c / common.h`
- Contains definitions of the following functions
  - bmi088_mm_interface_init()
  - bmi088_mm_check_rslt()
  - bmi088_mm_coines_deinit()

### read_sensor_data
- Configures sensor to read accelerometer and gyroscope data.
- Prints accel. data in `g` and gyro. data in `dps`
- Runs on PC, APP2.0 and APP3.0 microcontroller

### any_motion
- Configures sensor to generate interrupt when sensor is moved.
- Adjust threshold according to your application needs.
- Recommended ODR - 50 Hz, Threshold range - 0 to 1.5g
- Runs on PC, APP2.0 and APP3.0 microcontroller

### data_sync
- For information on data sync. feature, read [DataSync.md](https://github.com/BoschSensortec/BMI08x-Sensor-API/blob/master/DataSync.md)
- Requires shorting of Gyro. data ready interrupt pin with Accel. sync. input interrupt pin - Pins 21 and 22 on APP2.0 board
- Runs on APP2.0 and APP3.0 microcontroller

### high_g
- Configures sensor to generate interrupt when acceleration exceeds the set high-g threshold.
- Move the sensor upwards to create condition of high-g
- Increase hysteresis to reduce false triggers
- Recommended ODR - 200 Hz, Threshold range - 0 to 24g, Hysteresis - 0 to 3g
- Runs on PC, APP2.0 and APP3.0 microcontroller

### low_g
- Configures sensor to generate interrupt when acceleration value goes below the set low-g threshold.
- Move the sensor downwards (or) put in a state of free fall to create condition of low-g 
- Increase hysteresis to reduce false triggers
- Recommended ODR - 50 Hz, Threshold range - 0 to 1.5g, Hysteresis - 0 to 0.75g
- Runs on PC, APP2.0 and APP3.0 microcontroller

### no_motion
- Configures sensor to generate interrupt when sensor is stationary.
- Adjust threshold according to your application needs.
- Recommended ODR - 50 Hz, Threshold range - 0 to 1.5g
- Runs on PC, APP2.0 and APP3.0 microcontroller

### orientation
- Configures sensor to generate interrupt when sensor's orientation is changed.
- Shows the below outputs depending on the orientation
  - Portrait upright
  - Portrait upside down
  - Landscape left
  - Landscape right
  - Face up
  - Face down
- Runs on PC, APP2.0 and APP3.0 microcontroller

Commands to run tests on MCU:

To run in Command line :
---------------------------
1. mingw32-make clean all
2. mingw32-make all
3. mingw32-make TARGET=MCU_APP20 download
4. python -m serial.tools.list_ports -v   
     -> Lists available ports
	 -> Example :
	       COM3
               desc: Intel(R) Active Management Technology - SOL (COM3)
               hwid: PCI\VEN_8086&DEV_9D3D&SUBSYS_505317AA&REV_21\3&33FD14CA&0&B3
           COM20
               desc: Bosch Sensortec Application Board 2.0 (COM20)
               hwid: USB VID:PID=108C:AB2C SER=6 LOCATION=1-6.2
           2 ports found
5. python -m serial.tools.miniterm --dtr 1 -e --eol LF COM20 
    -> Select the port (here, COM20) which has 'desc' parameter with value 'Bosch Sensortec Application Board 2.0'
6. Give Quit command ( Ctrl + ] )

Note : 

-> Miniterm help command : python -m serial.tools.miniterm -h
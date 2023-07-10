# BMI088_ANYMOTION examples

## Examples overview

### `common`
- For the sake of simplicity of example code, repeated code is placed in `common.c / common.h`
- Contains definitions of the following functions
  - bmi088_anymotion_interface_init()
  - bmi088_anymotion_check_rslt()
  - bmi088_anymotion_coines_deinit()

### read_sensor_data
- Configures sensor to read accelerometer and gyroscope data.
- Prints accel. data in `g` and gyro. data in `dps`
- Runs on PC and APP3.0 microcontroller

### any_motion
- Configures sensor to generate interrupt when sensor is moved.
- Adjust threshold according to your application needs.
- Recommended ODR - 50 Hz, Threshold range - 0 to 1.5g
- Runs on PC, APP2.0 and APP3.0 microcontroller

---

Commands to run tests on MCU:

To run in Command line :
---------------------------
1. mingw32-make clean all
2. mingw32-make all
3. mingw32-make TARGET=MCU_APP30 download
4. python -m serial.tools.list_ports -v   
     -> Lists available ports
	 -> Example :
	       COM3
               desc: Intel(R) Active Management Technology - SOL (COM3)
               hwid: PCI\VEN_8086&DEV_9D3D&SUBSYS_505317AA&REV_21\3&33FD14CA&0&B3
           COM20
               desc: Bosch Sensortec Application Board 3.0 (COM20)
               hwid: USB VID:PID=108C:AB2C SER=6 LOCATION=1-6.2
           2 ports found
5. python -m serial.tools.miniterm --dtr 1 -e --eol LF COM20 
    -> Select the port (here, COM20) which has 'desc' parameter with value 'Bosch Sensortec Application Board 3.0'
6. Give Quit command ( Ctrl + ] )

Note : 

-> Miniterm help command : python -m serial.tools.miniterm -h

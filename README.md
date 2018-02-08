# BMI08X Sensor API

## Table of Contents
 - [Introduction](#Intro)
 - [Version and date](#Ver)
 - [Integration details](#Integration)
 - [Driver files information](#file)
 - [Sensor interfaces](#interface)
 - [Integration Examples](#examples)

### Introduction<a name=Intro></a>
This package contains Bosch Sensortec's BMI08X Sensor API.

### Version and date<a name=Ver></a> 

Driver files 	| Version |     Date        |
----------------|---------|---------------- |
_bmi088.c_      |  1.0.0  | 02 February, 2018|
_bmi088.h_      |  1.0.0  | 02 February, 2018|
_bmi08a.c_      |  1.0.0  | 02 February, 2018|
_bmi08g.c_      |  1.0.0  | 02 February, 2018|
_bmi08x_defs.h_ |  1.0.0  | 02 February, 2018|
_bmi08x.h_      |  1.0.0  | 02 February, 2018|


### Integration details<a name=Integration></a>
- Integrate _bmi088.c_,_bmi088.h_, _bmi08a.c_, _bmi08g.c_,_bmi08x_defs.h_ and _bmi08x.h_ in your project.

Enable the below macro in bmi08x_defs.h to use the BMI088 sensor feature 
/** \name enable bmi088 sensor */
 #ifndef BMI08X_ENABLE_BMI088
 #define BMI08X_ENABLE_BMI088       1
 #endif

- User has to include _bmi08x.h_ and _bmi088.h_ in the code to call sensor APIs as shown below :
``` c
#include "bmi08x.h"
#include "bmi088.h"

````
### Driver files information<a name=file></a>
- *_bmi088.c_*
   * This file has function definitions of bmi088 API interfaces.
- *_bmi088.h_*
   * This header file has necessary include files,bmi088 function declarations, required to make API calls.
- *_bmi08a.c_*
   * This file has function definitions of bmi08x accel generic API interfaces.
- *_bmi08g.c_*
   * This file has function definitions of bmi08x gyro generic API interfaces.   
- *_bmi08x.h_*
   * This header file has necessary include files,bmi08x function declarations, required to make API calls.
 - *_bmi08x_defs.h_*
   * This header file has necessary include files, macro definitions, typedefs and data structure definitions.
 
### Sensor interfaces<a name=interface></a>
- I2C interface
- SPI interface  
_Note: By default, the interface is I2C._

### Integration examples<a name=examples></a>
#### Initializing BMI088 sensors
 /* below code shows bmi088 integration steps */
 #include "bmi088.h"
To initialize BMI088 sensors, an instance of the bmi08x structure should be
created. The following parameters are required to be updated in the structure,
by the user, to initialize bmi088 sensors.

Parameters    | Details
--------------|-----------------------------------
_accel_id_    | Accel device address of I2C interface
_gyro_id_     | Gyro device address of I2C interface        
_intf_        | I2C or SPI 
_read_        | read through I2C/SPI interface
_write_       | write through I2C/SPI interface
_delay_ms_    | delay   

##### _Initialize through SPI interface_
``` c

int8_t rslt;

struct bmi08x_dev dev = {
        .accel_id = 0,
        .gyro_id = 0,
        .intf = BMI08X_SPI_INTF,  
        .read = user_spi_read,  
        .write = user_spi_write,  
        .delay_ms = user_delay_milli_sec
};

/* Initialize the SPI */

/* Initializing the bmi088 sensors the below function will Initialize both accel and gyro sensors*/
rslt = bmi088_init(&dev);

```

##### _Initialize through I2C interface_
``` c
/* I2c slave address depends on the hardware configuration for details please refer Data sheet*/

int8_t rslt;

struct bmi08x_dev dev = {
        .accel_id = BMI08X_ACCEL_I2C_ADDR_PRIMARY, /* User has define this macro depends on the I2C slave address */
        .gyro_id  = BMI08X_GYRO_I2C_ADDR_PRIMARY, /* User has define this macro depends on the I2C slave address */
        .intf = BMI08X_I2C_INTF,  
        .read = user_i2c_read,  
        .write = user_i2c_write,  
        .delay_ms = user_delay_milli_sec
};

/* Initialize the I2C */

/* Initializing the bmi088 sensors the below function will Initialize both accel and gyro sensors*/
rslt = bmi088_init(&dev)
	
```

#### Read Chip ID from the accel
``` c

int8_t rslt;
uint8_t data = 0;

/* Initialize the device instance as per the initialization example */

if(rslt == BMI08X_OK) 
{
		/* Read accel chip id */
		rslt = bmi08a_get_regs(BMI08X_ACCEL_CHIP_ID_REG, &data, 1, &dev);
}
			
```

#### Get the accel power mode
``` c
int8_t rslt;

/* Initialize the device instance as per the initialization example */

/* Read the accel power mode */
rslt = bmi08a_get_power_mode(&dev)	
/* power mode will be updated in the dev.accel_cfg.power */
	
```

#### Get the accelerometer configurations
``` c
int8_t rslt;

/* Initialize the device instance as per the initialization example */

/* Read the accel sensor config parameters (odr,bw,range) */
rslt = bmi08a_get_meas_conf(&dev)
/* config parameters will be updated in the dev.accel_cfg.odr,dev.accel_cfg.bw and dev.accel_cfg.range*/
	
```

#### Configuring the accelerometer
``` c

int8_t rslt;

/* Initialize the device instance as per the initialization example */

/* Assign the desired configurations */
dev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
dev.accel_cfg.odr = BMI08X_ACCEL_ODR_100_HZ;
dev.accel_cfg.range = BMI088_ACCEL_RANGE_4G;
dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;

rslt = bmi08a_set_power_mode(&dev);
/* Wait for 10ms to switch between the power modes - delay taken care inside the function*/

rslt = bmi08a_set_meas_conf(&dev);
	
```

#### Get accelerometer data
``` c

int8_t rslt;
struct bmi08x_sensor_data user_accel_bmi088;

/* Initialize the device instance as per the initialization example */

/* Read the sensor data into the sensor data instance */
rslt = bmi08a_get_data(&user_accel_bmi088, &dev);

```

#### Interrupt Configuring for accel data ready interrupt
``` c
/* Mapping data ready interrupt to interrupt channel */

int8_t rslt;
struct bmi08x_int_cfg int_config;

/* Initialize the device instance as per the initialization example */

/* Interrupt configurations */
int_config.accel_int_channel = BMI08X_INT_CHANNEL_1;
int_config.accel_int_type = BMI08X_ACCEL_DATA_RDY_INT;
int_config.accel_int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
int_config.accel_int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.accel_int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

/* Configure the controller port pin for the interrupt and assign the ISR */
	
/* Setting the interrupt configuration */
rslt = bmi08a_set_int_config(&int_config, &dev);


	
void interrupt_handler(void)
{
	/* ISR functionality */
}

/* Unmapping data ready interrupt to interrupt channel */

int8_t rslt;
struct bmi08x_int_cfg int_config;

/* Initialize the device instance as per the initialization example */

/* Interrupt configurations */
int_config.accel_int_channel = BMI08X_INT_CHANNEL_1;
int_config.accel_int_type = BMI08X_ACCEL_DATA_RDY_INT;
int_config.accel_int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
int_config.accel_int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.accel_int_pin_cfg.output_mode =BMI08X_INT_MODE_PUSH_PULL ;

/* Setting the interrupt configuration */
rslt = bmi08a_set_int_config(&int_config, &dev);

/* Configure the controller port pin for disabling the interrupt */

	
void interrupt_handler(void)
{
	/* ISR functionality */
}

```


#### Get the sensor time
``` c

int8_t rslt;
uint32_t user_sampling_time;

/* Initialize the device instance as per the initialization example */
	
/* Read the sensor time */
rslt = bmi08a_get_sensor_time(&dev, &user_sampling_time);
	
```

#### Read Chip ID from the gyro
``` c

int8_t rslt;
uint8_t data = 0;

/* Initialize the device instance as per the initialization example */

if(rslt == BMI08X_OK) 
{
		/* Read gyro chip id */
		rslt = bmi08g_get_regs(BMI08X_GYRO_CHIP_ID_REG, &data, 1, &dev);
}
			
```

#### Get the gyro power mode
``` c

int8_t rslt;

/* Initialize the device instance as per the initialization example */

/* Read the gyro power mode */
rslt = bmi08g_get_power_mode(&dev)
/* power mode will be updated in the dev.gyro_cfg.power */
	
```

#### Get the gyro sensor config
``` c

int8_t rslt;

/* Initialize the device instance as per the initialization example */

/* Read the gyro sensor config parameters (odr,bw,range) */
rslt = bmi08g_get_meas_conf(&dev)
/* config parameters will be updated in the dev.gyro_cfg.odr,dev.gyro_cfg.bw and dev.gyro_cfg.range */
	
```

#### Configuring the gyro
``` c

int8_t rslt;

/* Initialize the device instance as per the initialization example */
	
dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

rslt = bmi08g_set_power_mode(&dev);
/* Wait for 30ms to switch between the power modes - delay taken care inside the function*/
	
/* Assign the desired configurations */
dev.gyro_cfg.odr = BMI08X_GYRO_BW_23_ODR_200_HZ;
dev.gyro_cfg.range = BMI08X_GYRO_RANGE_1000_DPS;
dev.gyro_cfg.bw = BMI08X_GYRO_BW_23_ODR_200_HZ;

rslt = bmi08g_set_meas_conf(&dev);
	
```

#### Get gyro data
``` c

int8_t rslt;
struct bmi08x_sensor_data user_gyro_bmi088;

/* Initialize the device instance as per the initialization example */

/* Read the sensor data into the sensor data instance */
rslt = bmi08g_get_data(&user_gyro_bmi088, &dev);

```

#### Interrupt Configuring for gyro data ready interrupt
``` c
/* Mapping data ready interrupt to interrupt channel */

int8_t rslt;
struct bmi08x_int_cfg int_config;

/* Initialize the device instance as per the initialization example */

int_config.gyro_int_channel = BMI08X_INT_CHANNEL_3;
int_config.gyro_int_type = BMI08X_GYRO_DATA_RDY_INT;
int_config.gyro_int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
int_config.gyro_int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.gyro_int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

/* Configure the controller port pin for the interrupt and assign the ISR */

/* Setting the interrupt configuration */
rslt = bmi08g_set_int_config(&int_config, &dev);

	
void interrupt_handler(void)
{
	/* ISR functionality */
}

/* Unmapping data ready interrupt to interrupt channel */

/* Initialize the device instance as per the initialization example */

int_config.gyro_int_channel = BMI08X_INT_CHANNEL_3;
int_config.gyro_int_type = BMI08X_GYRO_DATA_RDY_INT;
int_config.gyro_int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
int_config.gyro_int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.gyro_int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

/* Setting the interrupt configuration */
rslt = bmi08g_set_int_config(&int_config, &dev);

/* Configure the controller port pin for disabling the interrupt */
	
void interrupt_handler(void)
{
	/* ISR functionality */
}


```
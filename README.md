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

Driver files 	| Version |    Date     |
----------------|---------|------------ |
_bmi085.c_      |  1.2.0  | 24 Aug, 2018|
_bmi085.h_      |  1.2.0  | 24 Aug, 2018|
_bmi088.c_      |  1.2.0  | 24 Aug, 2018|
_bmi088.h_      |  1.2.0  | 24 Aug, 2018|
_bmi08a.c_      |  1.2.0  | 24 Aug, 2018|
_bmi08g.c_      |  1.2.0  | 24 Aug, 2018|
_bmi08x_defs.h_ |  1.2.0  | 24 Aug, 2018|
_bmi08x.h_      |  1.2.0  | 24 Aug, 2018|


### Integration details<a name=Integration></a>
- Integrate _bmi085.c_,_bmi085.h_,_bmi088.c_,_bmi088.h_, _bmi08a.c_, _bmi08g.c_,_bmi08x_defs.h_ and _bmi08x.h_ in your project.

Enable the below macro in bmi08x_defs.h to use the BMI085 sensor feature
``` c
/** \name enable bmi085 sensor */
 #ifndef BMI08X_ENABLE_BMI085
 #define BMI08X_ENABLE_BMI085       1
 #endif
```
Enable the below macro in bmi08x_defs.h to use the BMI088 sensor feature 
``` c
/** \name enable bmi088 sensor */
 #ifndef BMI08X_ENABLE_BMI088
 #define BMI08X_ENABLE_BMI088       1
 #endif
```
- User has to include _bmi08x.h_ and _bmi085.h_/_bmi088.h_ in the code to call sensor APIs as shown below :
``` c
#include "bmi08x.h"
```
include the variant specific headers bmi085.h/bmi088.h

### Driver files information<a name=file></a>
- *_bmi085.c_*
   * This file has function definitions of bmi085 API interfaces.
- *_bmi085.h_*
   * This header file has necessary include files,bmi085 function declarations, required to make API calls.
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
#### Initializing BMI085 sensors
 /* below code shows bmi085 integration steps */
 #include "bmi085.h"
To initialize BMI085 sensors, an instance of the bmi08x structure should be created. The following parameters are required to be updated in the structure, by the user, to initialize bmi085 sensors.

Parameters    | Details
--------------|-------------------------------------------------------------------------------------
_accel_id_    | Accel device address of I2C interface (can be used to identify CSB1 pin in SPI mode)
_gyro_id_     | Gyro device address of I2C interface (can be used to identify CSB2 pin in SPI mode) 
_intf_        | I2C or SPI 
_read_        | read through I2C/SPI interface
_write_       | write through I2C/SPI interface
_delay_ms_    | delay   

As defined in _bmi08x_defs.h_, the _read_/_write_ functions must be implemented in such a way that they comply with the following template of a function declaration:

_typedef int8_t (*bmi08x_com_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);_

##### _Initialize through SPI interface_
In order to use SPI interface, the user has to implement dedicated SPI read/write functions, which could for example look like this:

_int8_t user_spi_write(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len);_

_int8_t user_spi_read(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len);_

Since the BMI08x sensor family has dedicated communication interfaces for gyro and accelerometer, two different chip select lines are required to retrieve all data from the sensor (see datasheet for details). The user can use the fields _accel_id_ and _gyro_id_ to provide an information to the generic SPI read/write function, which chip select to use. 

When the sensorAPI functions call the _int8_t user_spi_write_ and _int8_t user_spi_read_ functions, they will pass the fields _accel_id_ or _gyro_id_ to the _cs_pin_ parameter.

In the example below it is assumed that on the MCU platform of the user, the pin for CSB1 (chip select for accelerometer and temperature sensor data) is defined as MCU_GPIO_BMI08X_CSB1 and the pin for CSB2 (gyro chip select) is defined as MCU_GPIO_BMI08X_CSB2.
Thus the user need to provide the following values to the _bmi08x_dev_ structure and initialize SPI as follows.

``` c

int8_t rslt;

struct bmi08x_dev dev = {
        .accel_id = MCU_GPIO_BMI08X_CSB1,
        .gyro_id = MCU_GPIO_BMI08X_CSB2,
        .intf = BMI08X_SPI_INTF,  
        .read = user_spi_read,  
        .write = user_spi_write,  
        .delay_ms = user_delay_milli_sec
};

/* Initialize the SPI */

/* Initializing the bmi085 sensors the below function will Initialize both accel and gyro sensors*/
rslt = bmi085_init(&dev);

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

/* Initializing the bmi085 sensors the below function will Initialize both accel and gyro sensors*/
rslt = bmi085_init(&dev);
	
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
dev.accel_cfg.range = BMI085_ACCEL_RANGE_4G;
dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;

rslt = bmi08a_set_power_mode(&dev);
/* Wait for 10ms to switch between the power modes - delay taken care inside the function*/

rslt = bmi08a_set_meas_conf(&dev);
	
```

#### Get accelerometer data
``` c

int8_t rslt;
struct bmi08x_sensor_data user_accel_bmi085;

/* Initialize the device instance as per the initialization example */

/* Read the sensor data into the sensor data instance */
rslt = bmi08a_get_data(&user_accel_bmi085, &dev);

```

#### Interrupt Configuring for accel data ready interrupt
```c
/* Mapping data ready interrupt to interrupt channel */

int8_t rslt;
struct bmi08x_accel_int_channel_cfg int_config;

/* Initialize the device instance as per the initialization example */

/* Interrupt configurations */
int_config.int_channel = BMI08X_INT_CHANNEL_1;
int_config.int_type = BMI08X_ACCEL_DATA_RDY_INT;
int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

/* Configure the controller port pin for the interrupt and assign the ISR */
	
/* Setting the interrupt configuration */
rslt = bmi08a_set_int_config(&int_config, &dev);


	
void interrupt_handler(void)
{
	/* ISR functionality */
}

/* Unmapping data ready interrupt to interrupt channel */

int8_t rslt;
struct bmi08x_accel_int_channel_cfg int_config;

/* Initialize the device instance as per the initialization example */

/* Interrupt configurations */
int_config.int_channel = BMI08X_INT_CHANNEL_1;
int_config.int_type = BMI08X_ACCEL_DATA_RDY_INT;
int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

/* Setting the interrupt configuration */
rslt = bmi08a_set_int_config(&int_config, &dev);

/* Configure the controller port pin for disabling the interrupt */

	
void interrupt_handler(void)
{
	/* ISR functionality */
}
```
#### Interrupt Configuring for accel Anymotion interrupt
```c

/* Initialize the device instance as per the initialization example */

/*declare the anymotion configuration structure*/

struct bmi08x_anymotion_cfg anymotion_cfg;

struct bmi08x_accel_int_channel_cfg int_config;
uint8_t rslt;

/*! Max read/write length (maximum supported length is 32).
To be set by the user */
dev.read_write_len = 8;
/* Enabling Accel Anymotion interrupt */	
rslt = bmi085_apply_config_file(&dev);
	
/*configure the any motion parameters*/

anymotion_cfg.threshold = 0x44;
anymotion_cfg.nomotion_sel = 0x00;
anymotion_cfg.duration = 0x01;
anymotion_cfg.x_en = 0x01;
anymotion_cfg.y_en = 0x01;
anymotion_cfg.z_en = 0x01;
	
rslt=bmi085_configure_anymotion(anymotion_cfg, &dev);
	
/* Interrupt configurations */

int_config.int_channel = BMI08X_INT_CHANNEL_1;
int_config.int_type = BMI08X_ACCEL_DATA_RDY_INT;
int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

/* Configure the controller port pin for the interrupt and assign the ISR */
	
/* Setting the interrupt configuration */

rslt = bmi08a_set_int_config(&int_config, &dev);

void interrupt_handler(void)
{
	/* ISR functionality */
}

/* Disbaling Accel Anymotion interrupt */

struct bmi08x_anymotion_cfg anymotion_cfg;

anymotion_cfg.threshold = 0x44;
anymotion_cfg.nomotion_sel = 0x00;
anymotion_cfg.duration = 0x01;
anymotion_cfg.x_en = 0x00;
anymotion_cfg.y_en = 0x00;
anymotion_cfg.z_en = 0x00;

rslt=bmi085_configure_anymotion(anymotion_cfg, &dev);

if(rslt == BMI08X_OK)  {
	/* Interrupt configurations */
	int_config.int_channel = BMI08X_INT_CHANNEL_1;
	int_config.int_type = BMI08X_ACCEL_DATA_RDY_INT;
	int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
	int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
	int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
	
	/* Setting the interrupt configuration */
	rslt = bmi08a_set_int_config(&int_config, &dev);
}

```


#### Get the sensor time
```c

int8_t rslt;
uint32_t user_sampling_time;

/* Initialize the device instance as per the initialization example */
	
/* Read the sensor time */
rslt = bmi08a_get_sensor_time(&dev, &user_sampling_time);
	
```

#### Read Chip ID from the gyro
```c

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
struct bmi08x_sensor_data user_gyro_bmi085;

/* Initialize the device instance as per the initialization example */

/* Read the sensor data into the sensor data instance */
rslt = bmi08g_get_data(&user_gyro_bmi085, &dev);

```

#### Interrupt Configuring for gyro data ready interrupt
``` c

int8_t rslt;
struct bmi08x_gyro_int_channel_cfg int_config;

/* Initialize the device instance as per the initialization example */

/* Mapping data ready interrupt to interrupt channel */

int_config.int_channel = BMI08X_INT_CHANNEL_3;
int_config.int_type = BMI08X_GYRO_DATA_RDY_INT;
int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

/* Setting the interrupt configuration */
rslt = bmi08g_set_int_config(&int_config, &dev);

/* Configure the controller port pin for the interrupt and assign the ISR */
	
void interrupt_handler(void)
{
	/* ISR functionality */
}

/*Disabling gyro data ready interrupt*/

struct bmi08x_gyro_int_channel_cfg int_config;

/* Initialize the device instance as per the initialization example */

int_config.int_channel = BMI08X_INT_CHANNEL_3;
int_config.int_type = BMI08X_GYRO_DATA_RDY_INT;
int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

/* Setting the interrupt configuration */
rslt = bmi08g_set_int_config(&int_config, &dev);

/* Configure the controller port pin for disabling the interrupt */
	


# BMI085/BMI088 data synchronization

##	Introduction
BMI08X is a system-in-package inertial measurement unit which offers accurate acceleration and angular rate measurements. Due to system-in-package approach (two sensors in single package), the gyroscope and acceleration data is acquired in a non-synchronized manner. However, the synchronization between accelerometer and gyroscope can be easily achieved. This document describes how synchronization between accelerometer and gyroscope can be achieved in a typical application such as augmented or virtual reality.

To achieve data synchronization on BMI08X, the data ready interrupt signal from the gyroscope of the BMI08X needs to be connected to one of the interrupt pins of the BMI08X accelerometer (which can be configured as input pins). The internal signal processing unit of the accelerometer uses the data ready signal from the gyroscope to synchronize and interpolate the data of the accelerometer, considering the group delay of the sensors. The accelerometer part can then notify the host of available data. With this technique, it is possible to achieve synchronized data and provide accelerometer data at an ODR of 2 kHz.

_Note: data synchronization is designed for applications requiring high bandwidth, for which BMI085 is intended, but it works also with BMI088. However, for some applications it is desirable to have a very low bandwidth, and partly having different bandwidth settings for accelerometer and gyroscope. Here the synchronization feature does not make sense._

##	Concept
Synchronized data means that the acquisition of the gyroscope and accelerometer data is happening at the same time and the signals have same propagation time. The time between motion to register read-out depends on the physical propagation time mainly caused by signal filtering path. The synchronization between accelerometer and gyroscope data to a common point of time and a common group delay can be realized with the approach described in the following sections.

### Connection diagram (APP3.0)

```

       INT2 A   1   JP3   2   INT4 G           
     ------------- (-|-) ------------ 
```

For latency-critical multisensory applications, it is recommended to use SPI interface for fastest sensor data read (recommended SPI clock speed is >2MHz).

Further details in:

BMI088: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf

BMI085: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi085-ds001.pdf

### Configuring BMI08X for data synchronization

Include the bmi08x header 

```
 #include "bmi08x.h"
```

Update variant of bmi08_dev to BMI085_VARIANT to use the BMI085 sensor feature

```
 dev.variant = BMI085_VARIANT;
```

Update variant of bmi08_dev to BMI088_VARIANT to use the BMI088 sensor feature

```
 dev.variant = BMI088_VARIANT;
```

To initialize BMI08X for data synchronization, an instance of the bmi08 structure should be created. The following parameters are required to be updated in the structure by the user:


Parameters          | Details
--------------------|--------------------------------------------------------
_intf_ptr_accel_    | Interface pointer that can hold Accel device address
_intf_ptr_gyro_     | Interface pointer that can hold Gyro device address    
_intf_              | I2C or SPI 
_read_              | read through I2C/SPI interface
_write_             | write through I2C/SPI interface
_delay_us_          | delay in microseconds
_variant_           | BMI08X_VARIANT


##### _Initialize through SPI interface_ for BMI085

The following code is simplified code, for example no checking of return values is added, to make it easier to read the code.

int8_t rslt;

uint8_t acc_dev_addr = 0;
uint8_t gyro_dev_addr = 0;

struct bmi08_dev dev = {

        .intf_ptr_accel = &acc_dev_addr,
        .intf_ptr_gyro = &gyro_dev_addr,
        .intf = BMI08_SPI_INTF,  
        .read = user_spi_read,  
        .write = user_spi_write,  
        .delay_us = user_delay_milli_sec,
        .variant = BMI085_VARIANT
};

/* Initializing the bmi085 sensors the below functions */

/* To Initialize accel sensor */

rslt = bmi08xa_init(&dev);

/* To Initialize gyro sensor */

rslt = bmi08g_init(&dev);

/* Reset the accelerometer and wait for 1 ms - delay taken care inside the function */

rslt = bmi08a_soft_reset(&dev);

/* Read/write length */

dev.read_write_len = 32;

/* API uploads the bmi08x config file onto the device and wait for 150ms 
   to enable the data synchronization - delay taken care inside the function */

rslt = bmi08a_load_config_file(&dev);

/* Set accel power mode */

dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;

rslt = bmi08a_set_power_mode(&dev);

dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

bmi08g_set_power_mode(&dev);

/* Assign accel range setting*/

dev.accel_cfg.range = BMI085_ACCEL_RANGE_4G;

/* Assign gyro range setting*/

dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;

/* Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */

sync_cfg.mode = BMI08_ACCEL_DATA_SYNC_MODE_2000HZ;

rslt = bmi08xa_configure_data_synchronization(sync_cfg, &dev);

/* Set accel interrupt pin configuration*/

/* Configure host data ready interrupt */

int_config.accel_int_config_1.int_channel = BMI08_INT_CHANNEL_1;

int_config.accel_int_config_1.int_type = BMI08_ACCEL_SYNC_INPUT;

int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;

int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

/* Configure Accel syncronization input interrupt pin */

int_config.accel_int_config_2.int_channel = BMI08_INT_CHANNEL_2;

int_config.accel_int_config_2.int_type = BMI08_ACCEL_INT_SYNC_DATA_RDY;

int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;

int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

/* Set gyro interrupt pin configuration*/

int_config.gyro_int_config_1.int_channel = BMI08_INT_CHANNEL_3;

int_config.gyro_int_config_1.int_type = BMI08_GYRO_INT_DATA_RDY;

int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;

int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

int_config.gyro_int_config_2.int_channel = BMI08_INT_CHANNEL_4;

int_config.gyro_int_config_2.int_type = BMI08_GYRO_INT_DATA_RDY;

int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;

int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

/* Enable synchronization interrupt pin */

rslt = bmi08a_set_data_sync_int_config(&int_config, &dev);

#### Read out raw accel data

/* Declare an instance of the sensor data structure for accel */

static struct bmi08_sensor_data accel_bmi085;

rslt = bmi08a_get_data(&accel_bmi085, &dev);


#### Read out synchronized  data

/* Declare an instance of the sensor data structure for accel */

static struct bmi08_sensor_data accel_bmi085;

/* Declare an instance of the sensor data structure for gyro */

static struct bmi08_sensor_data gyro_bmi085;

rslt = bmi08a_get_synchronized_data(&accel_bmi085,&gyro_bmi085, &dev);

##### _Initialize through SPI interface_ for BMI088

The following code is simplified code, for example no checking of return values is added, to make it easier to read the code.

int8_t rslt;

uint8_t acc_dev_addr = 0;
uint8_t gyro_dev_addr = 0;

struct bmi08_dev dev = {

        .intf_ptr_accel = &acc_dev_addr,
        .intf_ptr_gyro = &gyro_dev_addr,
        .intf = BMI08_SPI_INTF,  
        .read = user_spi_read,  
        .write = user_spi_write,  
        .delay_us = user_delay_milli_sec,
        .variant = BMI088_VARIANT
};

/* Initializing the bmi088 sensors the below functions */

/* To Initialize accel sensor */

rslt = bmi08xa_init(&dev);

/* To Initialize gyro sensor */

rslt = bmi08g_init(&dev);

/* Reset the accelerometer and wait for 1 ms - delay taken care inside the function */

rslt = bmi08a_soft_reset(&dev);

/* Read/write length */

dev.read_write_len = 32;

/* API uploads the bmi08x config file onto the device and wait for 150ms 
   to enable the data synchronization - delay taken care inside the function */

rslt = bmi08a_load_config_file(&dev);

/* Set accel power mode */

dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;

rslt = bmi08a_set_power_mode(&dev);

dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

bmi08g_set_power_mode(&dev);

/* Assign accel range setting*/

dev.accel_cfg.range = BMI088_ACCEL_RANGE_6G;

/* Assign gyro range setting*/

dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;

/* Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */

sync_cfg.mode = BMI08_ACCEL_DATA_SYNC_MODE_2000HZ;

rslt = bmi08xa_configure_data_synchronization(sync_cfg, &dev);

/* Set accel interrupt pin configuration*/

/* Configure host data ready interrupt */

int_config.accel_int_config_1.int_channel = BMI08_INT_CHANNEL_1;

int_config.accel_int_config_1.int_type = BMI08_ACCEL_SYNC_INPUT;

int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;

int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

/* Configure Accel syncronization input interrupt pin */

int_config.accel_int_config_2.int_channel = BMI08_INT_CHANNEL_2;

int_config.accel_int_config_2.int_type = BMI08_ACCEL_INT_SYNC_DATA_RDY;

int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;

int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

/* Set gyro interrupt pin configuration*/

int_config.gyro_int_config_1.int_channel = BMI08_INT_CHANNEL_3;

int_config.gyro_int_config_1.int_type = BMI08_GYRO_INT_DATA_RDY;

int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;

int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

int_config.gyro_int_config_2.int_channel = BMI08_INT_CHANNEL_4;

int_config.gyro_int_config_2.int_type = BMI08_GYRO_INT_DATA_RDY;

int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08_DISABLE;

int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;

int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

/* Enable synchronization interrupt pin */

rslt = bmi08a_set_data_sync_int_config(&int_config, &dev);

#### Read out raw accel data

/* Declare an instance of the sensor data structure for accel */

static struct bmi08_sensor_data accel_bmi088;

rslt = bmi08a_get_data(&accel_bmi088, &dev);


#### Read out synchronized  data

/* Declare an instance of the sensor data structure for accel */

static struct bmi08_sensor_data accel_bmi088;

/* Declare an instance of the sensor data structure for gyro */

static struct bmi08_sensor_data gyro_bmi088;

rslt = bmi08a_get_synchronized_data(&accel_bmi088,&gyro_bmi088, &dev);

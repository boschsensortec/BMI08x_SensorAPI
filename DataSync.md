#### Configuring BMI085 for data synchronization

include the bmi08x header 
#include "bmi08x.h"
include the bmi085 variant header
#include "bmi085.h"

Enable the below macro in bmi08x_defs.h to use the BMI085 sensor feature 

``` c
/** \name enable bmi085 sensor */
 #ifndef BMI08X_ENABLE_BMI085
 #define BMI08X_ENABLE_BMI085       1
 #endif
```

To initialize BMI085 for data synchronization, an instance of the bmi08x structure 
should be created. The following parameters are required to be updated in the 
structure by the user:


Parameters    | Details
--------------|-----------------------------------
_accel_id_    | device address of I2C interface (if intf is BMI08X_I2C_INTF) 
_gyro_id_     | device address of I2C interface (if intf is BMI08X_I2C_INTF)      
_intf_        | I2C or SPI
_read_        | read through I2C/SPI interface
_write_       | write through I2C/SPI interface
_delay_ms_    | delay   


##### _Initialize through SPI interface_
``` c

struct bmi08x_dev bmi08xdev = {
        .accel_id = 0,
        .gyro_id = 0,
        .intf = BMI08X_SPI_INTF,  
        .read = user_spi_read,  
        .write = user_spi_write,  
        .delay_ms = user_delay_milli_sec
};


int8_t rslt;

/* Initilaize int config instance */
struct bmi08x_int_cfg int_config;


/* Initialize bmi085 sensors (accel & gyro)*/
rslt = bmi085_init(&bmi08xdev);

if (rslt == BMI08X_OK) {
	/* Reset the accelerometer */
	rslt = bmi08a_soft_reset(&bmi08xdev);
	/* Wait for 1 ms - delay taken care inside the function*/
}

/* Assign the accel settings,for data synchronization odr and bandwidth will be configured based on the data sync mode */

bmi08xdev.accel_cfg.range = BMI085_ACCEL_RANGE_4G;
bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;

/* Set the accel power mode */
rslt = bmi08a_set_power_mode(&bmi08xdev);
/* Wait for 10ms to switch to normal mode - delay taken care inside the function*/

if (rslt == BMI08X_OK) {
	/* Set the accel configuration */
	rslt = bmi08a_set_meas_conf(&bmi08xdev);
}

/*set gyro power mode */
	bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;
	
if (rslt == BMI08X_OK){
	rslt = bmi08g_set_power_mode(&bmi08xdev);
	/* Wait for 30ms to switch to normal mode -delay taken care inside the function */
}
	
/* Assign the gyro settings ,for data synchronization odr and bandwidth will be configured based on the data sync mode */
bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;

if (rslt == BMI08X_OK){
/* Set the gyro configuration */
rslt = bmi08g_set_meas_conf(&bmi08xdev);
}


/* Upload synchronization configuration  */
if (rslt == BMI08X_OK) {

    /*! Max read/write length (maximum supported length is 32).
     To be set by the user */
    bmi08xDev.read_write_len = 8;
	
	ret_value = bmi085_apply_config_file(&bmi08xDev);
	
	/* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
}

if (ret_value == BMI08X_OK)
{
		/*configure data synchronization mode */
		sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_2000HZ;
		
     	/*Enable data synchronization*/
       ret_value = bmi085_configure_data_synchronization(sync_cfg, &bmi08xDev);
       
		/*wait for 150 ms */
	  	bmi08xDev.delay_ms(BMI08X_ASIC_INIT_TIME_MS);
}

/* configure synchronization interrupt pins */

if (rslt == BMI085_OK) {
/*set accel interrupt pin configuration*/
/*configure host data ready interrupt */
int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;
int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
	
/*configure Accel syncronization input interrupt pin */
int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
int_config.accel_int_config_2.int_type = BMI08X_ACCEL_SYNC_DATA_RDY_INT;
int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

/*set gyro interrupt pin configuration*/
int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
int_config.gyro_int_config_1.int_type = BMI08X_GYRO_DATA_RDY_INT;
int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;
int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
		
int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
int_config.gyro_int_config_2.int_type = BMI08X_GYRO_DATA_RDY_INT;
int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

/* Enable synchronization interrupt pin */
rslt = bmi085_set_data_sync_int_config(&int_config, &bmi08xdev);

}
/* User pin configuration */

/* Need to configure the accel drdy gpio pin as input */

```

#### Read out raw accel data
```c

/* Declare an instance of the sensor data structure for accel */
static struct bmi08x_sensor_data accel_bmi085;

rslt = bmi08a_get_data(&accel_bmi085, &bmi08xdev);

```

#### Read out synchronized  data
```c
/* Declare an instance of the sensor data structure for accel */
static struct bmi08x_sensor_data accel_bmi085;
/* Declare an instance of the sensor data structure for gyro */
static struct bmi08x_sensor_data gyro_bmi085;

rslt = bmi085_get_synchronized_data(&accel_bmi085,&gyro_bmi085, &bmi08xdev);

```
#### Disable synchronization feature
```c

/*--------------------disable synchronization feature--------------------*/

/* Disable data synchronization */
int8_t rslt;
struct bmi08x_data_sync_cfg sync_cfg;
/*turn off the sync feature*/
sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_OFF;
rslt = bmi085_configure_data_synchronization(sync_cfg, &bmi08xdev);
/* Wait for 150ms to enable the data synchronization --delay taken care inside the function */
/* configure synchronization interrupt pins */

if (rslt == BMI08X_OK) {
	
int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;
int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT ;
int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
	
int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;
int_config.accel_int_config_2.int_type = BMI08X_ACCEL_SYNC_DATA_RDY_INT;
int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

/*set gyro interrupt pin configuration*/
int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;
int_config.gyro_int_config_1.int_type = BMI08X_GYRO_DATA_RDY_INT;
int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;
		
int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;
int_config.gyro_int_config_2.int_type = BMI08X_GYRO_DATA_RDY_INT;
int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;
int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;
int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

/* Disable synchronization interrupt pin */
rslt = bmi085_set_data_sync_int_config(&int_config, &bmi08xdev);	

}
```

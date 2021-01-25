

#include "bmi160_bsp.h"
#include "comm.h"

extern I2C_HandleTypeDef hi2c1;

signed char bmi160_rd(unsigned char dev_addr, unsigned char reg_addr, unsigned char *data, unsigned short int len)
{
	HAL_I2C_Master_Transmit(&hi2c1, dev_addr, &reg_addr, 1,0xFFFF);//写目标寄存器
	HAL_I2C_Master_Transmit(&hi2c1, dev_addr, data, len,0xFFFF);   //写目标数据至寄存器
	
	return 0;	
}
signed char bmi160_wr(unsigned char dev_addr, unsigned char reg_addr, unsigned char *data, unsigned short int len)
{
	HAL_I2C_Master_Receive(&hi2c1, dev_addr, &reg_addr, 1,0xFFFF);//写目标寄存器	
	HAL_I2C_Master_Receive(&hi2c1, dev_addr, data, len,0xFFFF);   //写目标数据至寄存器
	
	return 0;
}

unsigned char bmi160_bsp_init(struct bmi160_dev *me)
{
		me->id        = BMI160_I2C_ADDR;
		me->interface = BMI160_I2C_INTF;
		me->read      = bmi160_rd;
		me->write     = bmi160_wr;
		me->delay_ms  = HAL_Delay;//MS

		int8_t rslt = BMI160_OK;

		rslt = bmi160_init(me);
	
	  return rslt;
}


unsigned char bmi160_config_accel_gyro_sensors_in_normal_mode(struct bmi160_dev *me)
{
		int8_t rslt = BMI160_OK;

		/* Select the Output data rate, range of accelerometer sensor */
		me->accel_cfg.odr   = BMI160_ACCEL_ODR_1600HZ;
		me->accel_cfg.range = BMI160_ACCEL_RANGE_2G;
		me->accel_cfg.bw    = BMI160_ACCEL_BW_NORMAL_AVG4;
		/* Select the power mode of accelerometer sensor */
		me->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
		/* Select the Output data rate, range of Gyroscope sensor */
		me->gyro_cfg.odr    = BMI160_GYRO_ODR_3200HZ;
		me->gyro_cfg.range  = BMI160_GYRO_RANGE_2000_DPS;
		me->gyro_cfg.bw     = BMI160_GYRO_BW_NORMAL_MODE;
		/* Select the power mode of Gyroscope sensor */
		me->gyro_cfg.power  = BMI160_GYRO_NORMAL_MODE; 
		/* Set the sensor configuration */
		rslt = bmi160_set_sens_conf(me);	
	
	  return rslt;	
}

void bmi160_read_sensor_data(struct bmi160_dev *me)
{
		int8_t rslt = BMI160_OK;
	
		struct bmi160_sensor_data accel;
		struct bmi160_sensor_data gyro;

		/* To read only Accel data */
		rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &accel, NULL, me);
		/* To read only Gyro data */
		rslt = bmi160_get_sensor_data(BMI160_GYRO_SEL, NULL, &gyro, me);
		/* To read both Accel and Gyro data */
		bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, me);
		/* To read Accel data along with time */
		rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_TIME_SEL) , &accel, NULL, me);
		/* To read Gyro data along with time */
		rslt = bmi160_get_sensor_data((BMI160_GYRO_SEL | BMI160_TIME_SEL), NULL, &gyro, me);
		/* To read both Accel and Gyro data along with time*/
		bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, me);	
}

unsigned char bmi160_set_the_power_mode_of_sensors(struct bmi160_dev *me)
{
		int8_t rslt = BMI160_OK;

		/* Select the power mode */
		me->accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE; 
		me->gyro_cfg.power  = BMI160_GYRO_FASTSTARTUP_MODE; 

		/*  Set the Power mode  */
		rslt = bmi160_set_power_mode(me);

		/* Select the power mode */
		me->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
		me->gyro_cfg.power  = BMI160_GYRO_NORMAL_MODE; 

		/*  Set the Power mode  */
		rslt = bmi160_set_power_mode(me);	

	  return rslt;	
}

unsigned char bmi160_read_sensor_data_register(struct bmi160_dev *me)
{
		int8_t rslt      = BMI160_OK;
		uint8_t reg_addr = BMI160_CHIP_ID_ADDR;
		uint8_t data;
		uint16_t len = 1;
	
		rslt = bmi160_get_regs(reg_addr, &data, len, me);
	
	  return rslt;		
}

unsigned char bmi160_write_to_sensor_data_register(struct bmi160_dev *me)
{
		int8_t rslt      = BMI160_OK;
		uint8_t reg_addr = BMI160_INT_MOTION_1_ADDR;
		uint8_t data     = 20;
		uint16_t len     = 1;
	
		rslt = bmi160_set_regs(reg_addr, &data, len, me);	
	
	  return rslt;		
}

unsigned char bmi160_reset_the_device_use_soft_reset(struct bmi160_dev *me)
{
		int8_t rslt = BMI160_OK;
	
		rslt = bmi160_soft_reset(me);
	
	  return rslt;		
}


void bmi160_config_any_motion_interrupt(struct bmi160_dev *me)
{
		struct bmi160_int_settg int_config;

		/* Select the Interrupt channel/pin */
		int_config.int_channel = BMI160_INT_CHANNEL_1;// Interrupt channel/pin 1

		/* Select the Interrupt type */
		int_config.int_type = BMI160_ACC_ANY_MOTION_INT;// Choosing Any motion interrupt
		/* Select the interrupt channel/pin settings */
		int_config.int_pin_settg.output_en   = BMI160_ENABLE;// Enabling interrupt pins to act as output pin
		int_config.int_pin_settg.output_mode = BMI160_DISABLE;// Choosing push-pull mode for interrupt pin
		int_config.int_pin_settg.output_type = BMI160_DISABLE;// Choosing active low output
		int_config.int_pin_settg.edge_ctrl   = BMI160_ENABLE;// Choosing edge triggered output
		int_config.int_pin_settg.input_en    = BMI160_DISABLE;// Disabling interrupt pin to act as input
		int_config.int_pin_settg.latch_dur   = BMI160_LATCH_DUR_NONE;// non-latched output

		/* Select the Any-motion interrupt parameters */
		int_config.int_type_cfg.acc_any_motion_int.anymotion_en  = BMI160_ENABLE;// 1- Enable the any-motion, 0- disable any-motion 
		int_config.int_type_cfg.acc_any_motion_int.anymotion_x   = BMI160_ENABLE;// Enabling x-axis for any motion interrupt
		int_config.int_type_cfg.acc_any_motion_int.anymotion_y   = BMI160_ENABLE;// Enabling y-axis for any motion interrupt
		int_config.int_type_cfg.acc_any_motion_int.anymotion_z   = BMI160_ENABLE;// Enabling z-axis for any motion interrupt
		int_config.int_type_cfg.acc_any_motion_int.anymotion_dur = 0;// any-motion duration
		int_config.int_type_cfg.acc_any_motion_int.anymotion_thr = 20;// (2-g range) -> (slope_thr) * 3.91 mg, (4-g range) -> (slope_thr) * 7.81 mg, (8-g range) ->(slope_thr) * 15.63 mg, (16-g range) -> (slope_thr) * 31.25 mg 

		/* Set the Any-motion interrupt */
		bmi160_set_int_config(&int_config, me); /* sensor is an instance of the structure bmi160_dev  */	
}


void bmi160_config_flat_interrupt(struct bmi160_dev *me)
{
		struct bmi160_int_settg int_config;

		/* Select the Interrupt channel/pin */
		int_config.int_channel = BMI160_INT_CHANNEL_1;// Interrupt channel/pin 1

		/* Select the Interrupt type */
		int_config.int_type = BMI160_ACC_FLAT_INT;// Choosing flat interrupt
		/* Select the interrupt channel/pin settings */
		int_config.int_pin_settg.output_en   = BMI160_ENABLE;// Enabling interrupt pins to act as output pin
		int_config.int_pin_settg.output_mode = BMI160_DISABLE;// Choosing push-pull mode for interrupt pin
		int_config.int_pin_settg.output_type = BMI160_DISABLE;// Choosing active low output
		int_config.int_pin_settg.edge_ctrl   = BMI160_ENABLE;// Choosing edge triggered output
		int_config.int_pin_settg.input_en    = BMI160_DISABLE;// Disabling interrupt pin to act as input
		int_config.int_pin_settg.latch_dur   = BMI160_LATCH_DUR_NONE;// non-latched output

		/* Select the Flat interrupt parameters */
		int_config.int_type_cfg.acc_flat_int.flat_en        = BMI160_ENABLE;// 1-enable, 0-disable the flat interrupt
		int_config.int_type_cfg.acc_flat_int.flat_theta     = 8;// threshold for detection of flat position in range from 0�� to 44.8��.
		int_config.int_type_cfg.acc_flat_int.flat_hy        = 1;// Flat hysteresis
		int_config.int_type_cfg.acc_flat_int.flat_hold_time = 1;// Flat hold time (0 -> 0 ms, 1 -> 640 ms, 2 -> 1280 ms, 3 -> 2560 ms)

		/* Set the Flat interrupt */
		bmi160_set_int_config(&int_config, me); /* sensor is an instance of the structure bmi160_dev */	
}


void bmi160_config_step_detector_interrupt(struct bmi160_dev *me)
{
		struct bmi160_int_settg int_config;

		/* Select the Interrupt channel/pin */
		int_config.int_channel = BMI160_INT_CHANNEL_1;// Interrupt channel/pin 1

		/* Select the Interrupt type */
		int_config.int_type = BMI160_STEP_DETECT_INT;// Choosing Step Detector interrupt
		/* Select the interrupt channel/pin settings */
		int_config.int_pin_settg.output_en = BMI160_ENABLE;// Enabling interrupt pins to act as output pin
		int_config.int_pin_settg.output_mode = BMI160_DISABLE;// Choosing push-pull mode for interrupt pin
		int_config.int_pin_settg.output_type = BMI160_ENABLE;// Choosing active High output
		int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;// Choosing edge triggered output
		int_config.int_pin_settg.input_en = BMI160_DISABLE;// Disabling interrupt pin to act as input
		int_config.int_pin_settg.latch_dur =BMI160_LATCH_DUR_NONE;// non-latched output

		/* Select the Step Detector interrupt parameters, Kindly use the recommended settings for step detector */
		int_config.int_type_cfg.acc_step_detect_int.step_detector_mode = BMI160_STEP_DETECT_NORMAL;
		int_config.int_type_cfg.acc_step_detect_int.step_detector_en = BMI160_ENABLE;// 1-enable, 0-disable the step detector

		/* Set the Step Detector interrupt */
		bmi160_set_int_config(&int_config, me); /* sensor is an instance of the structure bmi160_dev */	
}


unsigned char bmi160_user_space(struct bmi160_dev *me)
{
		int8_t rslt = BMI160_OK;
		uint8_t step_enable = 1;//enable the step counter

		rslt = bmi160_set_step_counter(step_enable,me);	
	
	  return rslt;	
}


//### ISR
//``` c
//int8_t rslt = BMI160_OK;
//uint16_t step_count = 0;//stores the step counter value

//rslt = bmi160_read_step_counter(&step_count,  &sensor);
//```

//### Unmapping Interrupt
//#### Example for unmapping Step Detector Interrupt
//``` c
//struct bmi160_int_settg int_config;

///* Deselect the Interrupt channel/pin */
//int_config.int_channel = BMI160_INT_CHANNEL_NONE;
///* Select the Interrupt type */
//int_config.int_type = BMI160_STEP_DETECT_INT;// Choosing Step Detector interrupt
///* Set the Step Detector interrupt */
//bmi160_set_int_config(&int_config, &sensor); /* sensor is an instance of the structure bmi160_dev */
//```

//### Reading interrupt status
//#### Example for reading interrupt status for step detector
//``` c
//union bmi160_int_status interrupt;
//enum bmi160_int_status_sel int_status_sel;

///* Interrupt status selection to read all interrupts */
//int_status_sel = BMI160_INT_STATUS_ALL;
//rslt = bmi160_get_int_status(int_status_sel, &interrupt, &sensor);
//if (interrupt.bit.step)  printf("Step detector interrupt occured\n");
//```




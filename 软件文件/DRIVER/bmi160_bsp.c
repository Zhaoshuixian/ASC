

#include "bmi160_bsp.h"
#include "comm.h"

extern I2C_HandleTypeDef hi2c1;

//传感器读操作
signed char bmi160_rd(unsigned char dev_addr, unsigned char reg_addr, unsigned char *data, unsigned short int len)
{

	//数据帧结构操作(阻塞函数)
	HAL_I2C_Master_Transmit(&hi2c1,dev_addr,&reg_addr,1,0xFFFF);//1.发起读操作 ,写目标寄存器命令
	HAL_I2C_Master_Receive(&hi2c1,dev_addr,data,len,0xFFFF);    //2.接收目标寄存器值数据
	
	return 0;	
}

//传感器写操作
signed char bmi160_wr(unsigned char dev_addr, unsigned char reg_addr, unsigned char *data, unsigned short int len)
{
	#define MAX_TX_SIZE (64) //设置最大传输字节数
	
	unsigned char tx_msg[MAX_TX_SIZE]={reg_addr};
	
	if(MAX_TX_SIZE<len)//传输数据长度超过缓存长度
	{
		return -1;
	}
	tx_msg[0]=reg_addr;
	memcpy(&tx_msg[1],data,len);//合并数据至MSG缓存区,按照一帧格式传输
	//数据帧结构操作(阻塞函数)
	HAL_I2C_Master_Transmit(&hi2c1,dev_addr,&tx_msg[0],len+1,0xFFFF);   //寄存器地址和数据连续写入

	return 0;
}

/*
调试注意事项

1. 默认开机后bmi160进入suspend mode，此时bmi160的加速度及陀螺仪功能均处于未工作状态。
   需配置R0x7e寄存器使得加速度及陀螺仪功能进入正常工作（数据采样）模式。
2. 每次进行加速度数据检测前，请先执行i2c_write_byte(0x7e,0x11)，使得加速度模块进入normal工作模式；
3. 每次进行陀螺仪数据检测前，请先执行i2c_write_byte(0x7e,0x15) 使得陀螺仪模块进入normal工作模式；

3.控制寄存器

地址：0x7e。寄存器名：CMD。默认值：0x00。该寄存器可以读也可以写。

我们通过向该地址写入不同的值来控制加速度或者陀螺仪的工作模式。

0x11：通过写入该命令值，可以使加速度模块切换到正常工作模式。
0x15：通过写入该命令值，可以使陀螺仪模块切换到正常工作模式。

*/

//传感器初始化
unsigned char bmi160_bsp_init(struct bmi160_dev *me)
{
	me->id        = BMI160_I2C_ADDR;//配置设备地址 :BMI160传感器的I2C设备地址是0x68(当SDO脚接地)/0x69(当SDO脚拉高)。
	me->interface = BMI160_I2C_INTF;//配置I2C模式
	me->read      = bmi160_rd;      //I2C读写操作函数
	me->write     = bmi160_wr;      //I2C读写操作函数
	me->delay_ms  = HAL_Delay;      //MS级别延时函数

	int8_t rslt = BMI160_OK;
	
	rslt = bmi160_init(me);        //初始化配置

	return rslt;
}

//配置加速度传感器和陀螺仪传感器
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

//读取传感器数据
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

//设置传感器电源模式
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

//读传感器寄存器数据
unsigned char bmi160_read_sensor_data_register(struct bmi160_dev *me)
{
		int8_t rslt      = BMI160_OK;
		uint8_t reg_addr = BMI160_CHIP_ID_ADDR;
		uint8_t data;
		uint16_t len = 1;
	
		rslt = bmi160_get_regs(reg_addr, &data, len, me);
	
	  return rslt;		
}

//写传感器寄存器数据
unsigned char bmi160_write_to_sensor_data_register(struct bmi160_dev *me)
{
		int8_t rslt      = BMI160_OK;
		uint8_t reg_addr = BMI160_INT_MOTION_1_ADDR;
		uint8_t data     = 20;
		uint16_t len     = 1;
	
		rslt = bmi160_set_regs(reg_addr, &data, len, me);	
	
	  return rslt;		
}

//软件复位传感器设备
unsigned char bmi160_reset_the_device_use_soft_reset(struct bmi160_dev *me)
{
		int8_t rslt = BMI160_OK;
	
		rslt = bmi160_soft_reset(me);
	
	  return rslt;		
}

//配置传感器任意运动中断
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

//配置传感器平面中断
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

//配置传感器步进检测器中断
void bmi160_config_step_detector_interrupt(struct bmi160_dev *me)
{
		struct bmi160_int_settg int_config;

		/* Select the Interrupt channel/pin */
		int_config.int_channel = BMI160_INT_CHANNEL_1;// Interrupt channel/pin 1

		/* Select the Interrupt type */
		int_config.int_type = BMI160_STEP_DETECT_INT;// Choosing Step Detector interrupt
		/* Select the interrupt channel/pin settings */
		int_config.int_pin_settg.output_en   = BMI160_ENABLE;// Enabling interrupt pins to act as output pin
		int_config.int_pin_settg.output_mode = BMI160_DISABLE;// Choosing push-pull mode for interrupt pin
		int_config.int_pin_settg.output_type = BMI160_ENABLE;// Choosing active High output
		int_config.int_pin_settg.edge_ctrl   = BMI160_ENABLE;// Choosing edge triggered output
		int_config.int_pin_settg.input_en    = BMI160_DISABLE;// Disabling interrupt pin to act as input
		int_config.int_pin_settg.latch_dur   = BMI160_LATCH_DUR_NONE;// non-latched output

		/* Select the Step Detector interrupt parameters, Kindly use the recommended settings for step detector */
		int_config.int_type_cfg.acc_step_detect_int.step_detector_mode = BMI160_STEP_DETECT_NORMAL;
		int_config.int_type_cfg.acc_step_detect_int.step_detector_en = BMI160_ENABLE;// 1-enable, 0-disable the step detector

		/* Set the Step Detector interrupt */
		bmi160_set_int_config(&int_config, me); /* sensor is an instance of the structure bmi160_dev */	
}

/*
要配置步进计数器，用户需要按照上节所述配置步进检测器中断。
配置步进检测器后，请参见以下有关用户空间和ISR的代码片段要配置步进计数器，
用户需要按照上节所述配置步进检测器中断。配置步进检测器后，
请参见以下有关用户空间和ISR的代码片段。
*/
//
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




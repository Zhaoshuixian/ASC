

#include "bmi160_bsp.h"
#include "comm.h"

extern I2C_HandleTypeDef hi2c1;
struct bmi160_dev sensor_bmi160;
//IIC读操作
signed char bmi160_rd(unsigned char dev_addr, unsigned char reg_addr, unsigned char *data, unsigned short int len)
{
	if(0==data || 0==len)
	{
		return -1;
	}
	if( HAL_I2C_Mem_Read(&hi2c1,dev_addr,reg_addr,I2C_MEMADD_SIZE_8BIT,data,len,1000) == HAL_OK )
	{
			return 0;
	}
	else
	{
		return -1;
	}
}

//IIC写操作
signed char bmi160_wr(unsigned char dev_addr, unsigned char reg_addr, unsigned char *data, unsigned short int len)
{
	if(0==data || 0==len)
	{
		return -1;
	}	
	if(HAL_I2C_Mem_Write(&hi2c1,dev_addr,reg_addr,I2C_MEMADD_SIZE_8BIT,data,len,1000) == HAL_OK )
	{
		return 0;
	}
	else
	{
		return -1;
	}
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
	me->accel_cfg.range = BMI160_ACCEL_RANGE_2G; //2g
	me->accel_cfg.bw    = BMI160_ACCEL_BW_NORMAL_AVG4;
	/* Select the power mode of accelerometer sensor */
	me->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE; //0x7E -> 0x11
	/* Select the Output data rate, range of Gyroscope sensor */
	me->gyro_cfg.odr    = BMI160_GYRO_ODR_3200HZ;
	me->gyro_cfg.range  = BMI160_GYRO_RANGE_2000_DPS;//2000dps
	me->gyro_cfg.bw     = BMI160_GYRO_BW_NORMAL_MODE;
	/* Select the power mode of Gyroscope sensor */
	me->gyro_cfg.power  = BMI160_GYRO_NORMAL_MODE;  //0x7E -> 0x15
	/* Set the sensor configuration */
	rslt = bmi160_set_sens_conf(me);	
	
	  return rslt;	
}

/*
***中值滤波算法
*/
unsigned int midval_filter(unsigned int *sample_buff,unsigned char sample_num) 
{
	unsigned int temp;

	for(unsigned char j=0;j<sample_num-1;j++) 
	{
	  for (unsigned char i=0;i<sample_num-j;i++) 
	  { 
			if ( sample_buff[i]>sample_buff[i+1] )
			{ 
				temp = sample_buff[i]; 
				sample_buff[i] = sample_buff[i+1]; //相邻数据相互交换 
				sample_buff[i+1] = temp; //（把 N 次采样值按大小排列，最后数据依次从小到大排列）
			} 
		}
	} 
	return sample_buff[(sample_num-1)/2];//取中间值为本次有效值 
}

/*
***均值算法
*/
unsigned int avg_filter(unsigned int *sample_buff,unsigned char sample_num) 
{ 
	unsigned int sum = 0; 
	
	for (unsigned char i=0;i<sample_num;i++) 
	{ 
		sum += sample_buff[i];
	}
	return (sum/sample_num);//先求其和，再求其平均 
}

/*
***加速度数据转换
*/
unsigned int accel_data_convert(unsigned int raw_data)
{
	
	if(raw_data>0x7FFF)
	{
	  raw_data = -(0xFFFF-raw_data);
	}
	
	raw_data = (raw_data*9.8)/(0x8000/2);//当量程为±2g时，转换为g/s的加速度换算公式(9.8)

	
	return  raw_data;
}

/*
***陀螺仪数据转换
*/
unsigned int gyro_data_convert(unsigned int raw_data)
{
	if(raw_data>0x7FFF)
	{
			raw_data = -(0xFFFF-raw_data);
	}
	
	raw_data = (raw_data*2000)/0x7FFF;// range为2000dps时，转换为角速度°/s的公式	
	
	return raw_data;
}

//读取传感器数据
void bmi160_read_sensor_data(struct bmi160_dev *me)
{
	#define N (21) //连续采样 N 次（N 取奇数）
	
	int8_t rslt = BMI160_OK;
	
	struct bmi160_sensor_data accel;
	struct bmi160_sensor_data gyro;

#if DEBIG_MODE
	/* To read only Accel data */
	rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &accel, NULL, me);
	printf("Accel data is %d...\r\n",rslt);
	/* To read only Gyro data */
	rslt = bmi160_get_sensor_data(BMI160_GYRO_SEL, NULL, &gyro, me);
	printf("Gyro data is %d...\r\n",rslt);	
	/* To read both Accel and Gyro data */
	bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, me);
	/* To read Accel data along with time */
	rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_TIME_SEL) , &accel, NULL, me);

	/* To read Gyro data along with time */
	rslt = bmi160_get_sensor_data((BMI160_GYRO_SEL | BMI160_TIME_SEL), NULL, &gyro, me);
#endif		

#if 0
	unsigned  int accel_xsample_buff[N];
	unsigned  int accel_ysample_buff[N];
	unsigned  int accel_zsample_buff[N];
	
	unsigned  int gyro_xsample_buff[N];	
	unsigned  int gyro_ysample_buff[N];
	unsigned  int gyro_zsample_buff[N];	
	
  for(unsigned char i= 0;i< N;i++)//连续读取
	{
		/* To read both Accel and Gyro data along with time*///同时读取
		bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, me);	

		accel_xsample_buff[i] = accel.x;
		accel_ysample_buff[i] = accel.y;	
		accel_zsample_buff[i] = accel.z;	

		gyro_xsample_buff[i] = gyro.x;
		gyro_ysample_buff[i] = gyro.y;
		gyro_zsample_buff[i] = gyro.z;		
	}
#if 0	
	avg_filter(accel_xsample_buff,N) ;//经过滤波
	avg_filter(accel_ysample_buff,N) ;//经过滤波
	avg_filter(accel_zsample_buff,N) ;//经过滤波
	
	avg_filter(gyro_xsample_buff,N) ;//经过滤波
	avg_filter(gyro_ysample_buff,N) ;//经过滤波
	avg_filter(gyro_zsample_buff,N) ;//经过滤波	
#else	
	accel.x = midval_filter(accel_xsample_buff,N) ;//经过滤波
	accel.y = midval_filter(accel_ysample_buff,N) ;//经过滤波
	accel.z = midval_filter(accel_zsample_buff,N) ;//经过滤波

	gyro.x = midval_filter(gyro_xsample_buff,N) ;//经过滤波
	gyro.y = midval_filter(gyro_ysample_buff,N) ;//经过滤波
	gyro.z = midval_filter(gyro_zsample_buff,N) ;//经过滤波	
#endif
#else
	unsigned  int accel_sample_buff[3][N];
	unsigned  int gyro_sample_buff[3][N];

  for(unsigned char i= 0;i< N;i++)//连续读取
	{
		/* To read both Accel and Gyro data along with time*///同时读取
		bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, me);	

		accel_sample_buff[0][i] = accel.x;
		accel_sample_buff[1][i] = accel.y;	
		accel_sample_buff[2][i] = accel.z;	

		gyro_sample_buff[0][i] = gyro.x;
		gyro_sample_buff[1][i] = gyro.y;
		gyro_sample_buff[2][i] = gyro.z;		
	}
	accel.x = midval_filter(accel_sample_buff[0],N) ;//经过滤波
	accel.y = midval_filter(accel_sample_buff[1],N) ;//经过滤波
	accel.z = midval_filter(accel_sample_buff[2],N) ;//经过滤波

	gyro.x = midval_filter(gyro_sample_buff[0],N) ;//经过滤波
	gyro.y = midval_filter(gyro_sample_buff[1],N) ;//经过滤波
	gyro.z = midval_filter(gyro_sample_buff[2],N) ;//经过滤波	
#endif
//	accel.x =(float)accel_data_convert(accel.x);
//	accel.y =(float)accel_data_convert(accel.y);
//	accel.z =(float)accel_data_convert(accel.z);
	
//	gyro.x =(float)gyro_data_convert(gyro.x);
//	gyro.y =(float)gyro_data_convert(gyro.y);
//	gyro.z =(float)gyro_data_convert(gyro.z);	
	printf("Accel data is x=%d,y=%d,z=%d...\r\n",accel.x,accel.y,accel.z);	
	printf("Gyro data is x=%d,y=%d,z=%d...\r\n",gyro.x,gyro.y,gyro.z);			
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

/*
***BMI160初始化配置
*/
void bmi160_config_init(void)
{
	if(BMI160_OK == bmi160_config_accel_gyro_sensors_in_normal_mode(&sensor_bmi160))
	{

	}
}




#ifndef __BMI160_BSP_H__
#define __BMI160_BSP_H__

#include "bmi160.h"

unsigned char bmi160_bsp_init(struct bmi160_dev *me);
unsigned char bmi160_config_accel_gyro_sensors_in_normal_mode(struct bmi160_dev *me);
void bmi160_read_sensor_data(struct bmi160_dev *me);
unsigned char bmi160_set_the_power_mode_of_sensors(struct bmi160_dev *me);
unsigned char bmi160_read_sensor_data_register(struct bmi160_dev *me);
unsigned char bmi160_write_to_sensor_data_register(struct bmi160_dev *me);
unsigned char bmi160_reset_the_device_use_soft_reset(struct bmi160_dev *me);
void bmi160_config_any_motion_interrupt(struct bmi160_dev *me);
void bmi160_config_flat_interrupt(struct bmi160_dev *me);
void bmi160_config_step_detector_interrupt(struct bmi160_dev *me);
unsigned char bmi160_user_space(struct bmi160_dev *me);

#endif



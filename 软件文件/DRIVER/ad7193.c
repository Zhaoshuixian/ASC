
#include "ad7193.h"		// ad7193 definitions.

unsigned char currentPolarity = 0;
unsigned char currentGain     = 1;

/***************************************************************************//**
 * @brief Checks if the AD7139 part is present.
 *
 * @return status - Indicates if the part is present or not.
*******************************************************************************/

unsigned char ad7193_init(void)
{
		unsigned char status = 0;//OK
		unsigned char regVal = 0;

		ad7193_reset();//保证调试上电后同步。
		HAL_Delay(1);//上电自动复位后，延时>=500us

		regVal = ad7193_get_register_value(AD7193_REG_ID, 1, 1);//读取配置寄存器值

		if((regVal & AD7193_ID_MASK) != ID_AD7193)  status = 1;//判断IC是否挂载

		ad7193_reset(); //读取挂载成功，则再次复位IC
		HAL_Delay(1);	

		return status;
}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue - Data value to write.
 * @param bytesNumber - Number of bytes to be written.
 * @param modifyCS - Allows Chip Select to be modified.
 *
 * @return none.
*******************************************************************************/

void ad7193_set_register_value(unsigned char registerAddress,unsigned long registerValue,unsigned char bytesNumber,unsigned char modifyCS)
{
    unsigned char writeCommand[5] = {0, 0, 0, 0, 0};
    unsigned char* dataPointer    = (unsigned char*)&registerValue;
    unsigned char bytesNr         = bytesNumber;
    
    writeCommand[0] = AD7193_COMM_WRITE|AD7193_COMM_ADDR(registerAddress); //命令
		
    while(bytesNr > 0)
    {
        writeCommand[bytesNr] = *dataPointer;
        dataPointer ++;
        bytesNr --;
    }

    SPI_Write(AD7193_SLAVE_ID * modifyCS, writeCommand, bytesNumber + 1);	
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 * @param bytesNumber - Number of bytes that will be read.
 * @param modifyCS    - Allows Chip Select to be modified.
 *
 * @return buffer - Value of the register.
*******************************************************************************/

unsigned long ad7193_get_register_value(unsigned char registerAddress,unsigned char bytesNumber,unsigned char modifyCS)
{
    unsigned char registerWord[5] = {0, 0, 0, 0, 0}; 
    unsigned long buffer          = 0x0;
    unsigned char i               = 0;
    
    registerWord[0] = AD7193_COMM_READ |AD7193_COMM_ADDR(registerAddress);//寄存器地址

    SPI_Read(AD7193_SLAVE_ID * modifyCS, registerWord, bytesNumber + 1);// registerWord[0]/registerWord[1]

    for(i = 1; i < bytesNumber + 1; i++) 
    {
        buffer = (buffer << 8) + registerWord[i];//registerWord[1]
    }
    
    return buffer;
}

/***************************************************************************//**
 * @brief Resets the device.
 *
 * @return none.
*******************************************************************************/

void ad7193_reset(void)
{
    unsigned char registerWord[6] = {0, 0, 0, 0, 0, 0};
    
    registerWord[0] = 0xFF;
    registerWord[1] = 0xFF;
    registerWord[2] = 0xFF;
    registerWord[3] = 0xFF;
    registerWord[4] = 0xFF;
    registerWord[5] = 0xFF;

    SPI_Write(AD7193_SLAVE_ID, registerWord, 6);

}

/***************************************************************************//**
 * @brief Set device to idle or power-down.
 *
 * @param pwrMode - Selects idle mode or power-down mode.
 *                  Example: 0 - power-down
 *                           1 - idle
 *
 * @return none.
*******************************************************************************/

void ad7193_setpower(unsigned char pwrMode)
{
     unsigned long oldPwrMode = 0x0;
     unsigned long newPwrMode = 0x0; 
 
     oldPwrMode = ad7193_get_register_value(AD7193_REG_MODE, 3, 1);
     oldPwrMode &= ~(AD7193_MODE_SEL(0x7));
     newPwrMode = oldPwrMode | AD7193_MODE_SEL((pwrMode * (AD7193_MODE_IDLE))|(!pwrMode * (AD7193_MODE_PWRDN)));
     ad7193_set_register_value(AD7193_REG_MODE, newPwrMode, 3, 1);  
}

/***************************************************************************//**
 * @brief Waits for RDY pin to go low.
 *
 * @return none.
*******************************************************************************/
void ad7193_wait_ready_go_low(void)
{
	//当获得转换结果时，DOUT/RDY便会变为低电平
    while(AD7193_RDY_STATE){}
}

/***************************************************************************//**
 * @brief Selects the channel to be enabled.
 *
 * @param channel - Selects a channel.
 *                  Example: ad7193_CH_0 - AIN1(+) - AIN2(-);  (Pseudo = 0)
 *                           ad7193_CH_1 - AIN3(+) - AIN4(-);  (Pseudo = 0)
 *                           ad7193_TEMP - Temperature sensor
 *                           ad7193_SHORT - AIN2(+) - AIN2(-); (Pseudo = 0)
 *  
 * @return none.
*******************************************************************************/
void ad7193_channel_select(unsigned short channel)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;   
     
    oldRegValue = ad7193_get_register_value(AD7193_REG_CONF, 3, 1);//读取目标寄存器值
    oldRegValue &= ~(AD7193_CONF_CHAN(0x3FF));
    newRegValue = oldRegValue | AD7193_CONF_CHAN(1 << channel);  //设置新值 
    ad7193_set_register_value(AD7193_REG_CONF, newRegValue, 3, 1); //将新值重新写入寄存器
}

/***************************************************************************//**
 * @brief Performs the given calibration to the specified channel.
 *
 * @param mode - Calibration type.
 * @param channel - Channel to be calibrated.
 *
 * @return none.
*******************************************************************************/
void ad7193_calibrate(unsigned char mode, unsigned char channel)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;
    
    ad7193_channel_select(channel);//选择目标通道
    oldRegValue = ad7193_get_register_value(AD7193_REG_MODE, 3, 1);//读取目标寄存器值
    oldRegValue &= ~AD7193_MODE_SEL(0x7);
    newRegValue = oldRegValue | AD7193_MODE_SEL(mode);
	
    PMOD1_CS_LOW; 
    ad7193_set_register_value(AD7193_REG_MODE, newRegValue, 3, 0); //将新值重新写入寄存器
    ad7193_wait_ready_go_low();
    PMOD1_CS_HIGH;
}

/***************************************************************************//**
 * @brief Selects the polarity of the conversion and the ADC input range.
 *
 * @param polarity - Polarity select bit. 
                     Example: 0 - bipolar operation is selected.
                              1 - unipolar operation is selected.
* @param range - Gain select bits. These bits are written by the user to select 
                 the ADC input range.     
 *

 * @return none.
*******************************************************************************/
void ad7193_range_setup(unsigned char polarity, unsigned char range)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;
    
    oldRegValue = ad7193_get_register_value(AD7193_REG_CONF,3, 1);//先读取配置寄存器的值
    oldRegValue &= ~(AD7193_CONF_UNIPOLAR |AD7193_CONF_GAIN(0x7));
	
    newRegValue = oldRegValue|(polarity * AD7193_CONF_UNIPOLAR)|AD7193_CONF_GAIN(range); //再重新配置寄存器的值
    ad7193_set_register_value(AD7193_REG_CONF, newRegValue, 3, 1);//写入目标寄存器内

    currentPolarity = polarity;
    currentGain = 1 << range;
}

/***************************************************************************//**
 * @brief Returns the result of a single conversion.
 *
 * @return regData - Result of a single analog-to-digital conversion.
*******************************************************************************/
unsigned long ad7193_single_conversion(void)
{
    unsigned long command = 0x0;
    unsigned long regData = 0x0;
	
    #if USE_EXT_CLOCK
			command = AD7193_MODE_SEL(AD7193_MODE_SINGLE)|AD7193_MODE_CLKSRC(AD7193_CLK_EXT_MCLK1_2)|AD7193_MODE_RATE(0x060);		
	  #else
			command = AD7193_MODE_SEL(AD7193_MODE_SINGLE)|AD7193_MODE_CLKSRC(AD7193_CLK_INT)|AD7193_MODE_RATE(0x060); 
	  #endif

    PMOD1_CS_LOW;
    ad7193_set_register_value(AD7193_REG_MODE, command, 3, 0); // CS is not modified.
    ad7193_wait_ready_go_low();//等待数据完成
    regData = ad7193_get_register_value(AD7193_REG_DATA, 3, 0);
    PMOD1_CS_HIGH;
    
    return regData;
}

/***************************************************************************//**
 * @brief Returns the average of several conversion results.
 *
 * @return samplesAverage - The average of the conversion results.
*******************************************************************************/
unsigned long ad7193_continuous_readavg(unsigned char sampleNumber)
{
    unsigned long samplesAverage = 0;
    unsigned long command        = 0;
    unsigned char count          = 0;
        
    #if USE_EXT_CLOCK
			command = AD7193_MODE_SEL(AD7193_MODE_SINGLE)|AD7193_MODE_CLKSRC(AD7193_CLK_EXT_MCLK1_2)|AD7193_MODE_RATE(0x060);		
	  #else
			command = AD7193_MODE_SEL(AD7193_MODE_SINGLE)|AD7193_MODE_CLKSRC(AD7193_CLK_INT)|AD7193_MODE_RATE(0x060); 
	  #endif
	
    PMOD1_CS_LOW;
	
    ad7193_set_register_value(AD7193_REG_MODE, command, 3, 0); // CS is not modified.
    for(count = 0; count < sampleNumber; count++)
    {
        ad7193_wait_ready_go_low();
        samplesAverage += ad7193_get_register_value(AD7193_REG_DATA, 3, 0); // CS is not modified.
    }
		
    PMOD1_CS_HIGH;
    samplesAverage = samplesAverage / sampleNumber;
    
    return samplesAverage;
}

/***************************************************************************//**
 * @brief Read data from temperature sensor and converts it to Celsius degrees.
 *
 * @return temperature - Celsius degrees.
*******************************************************************************/
unsigned long ad7193_temperature_read(void)
{
    unsigned long dataReg     = 0;
    unsigned long temperature = 0;    
    
    ad7193_range_setup(1, AD7193_CONF_GAIN_1); // Bipolar operation, 0 Gain.
    ad7193_channel_select(AD7193_CH_TEMP);
    dataReg = ad7193_single_conversion();
    dataReg -= 0x800000;
    temperature = dataReg / 2815;    // Kelvin Temperature
    temperature -= 273;              // Celsius Temperature
    
    return temperature;
}

/***************************************************************************//**
 * @brief Converts 24-bit raw data to volts.
 *
 * @param rawData - 24-bit data sample.
 * @param vRef - The value of the voltage reference used by the device.
 *
 * @return voltage - The result of the conversion expressed as volts.
*******************************************************************************/
float ad7193_convert_to_volts(unsigned long rawData, float vRef)
{
    float voltage = 0;
    
    if(currentPolarity == 0 )   // Bipolar mode
    {
        voltage = (((float)rawData / (1ul << 23)) - 1) * vRef / currentGain;
    }
    else                        // Unipolar mode
    {
        voltage = ((float)rawData * vRef) / (1ul << 24) / currentGain;
    }
    
    return voltage;
}

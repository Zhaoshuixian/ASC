
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
    unsigned char registerWord[6] = {0};

 /*
  对DIN输入写入一连串的1，可以复位串行接口。如果在至
	少40个串行时钟内持续向AD7193 DIN线路写入逻辑1，该
	串行接口便会复位。当产生软件错误或系统故障，继而导
	致接口时序错误时，这种方法可确保将接口复位到已知状
	态。复位使接口返回到期待对通信寄存器执行写操作的状
	态。该操作会将所有寄存器的内容复位到其上电值。复位
	后，用户应等待500 μs再访问串行接口
 */   
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
     oldPwrMode &= ~(AD7193_MODE_SEL(0x7));//清除模式选择位数据
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
    /*
    DOUT/RDY引脚也可用作数据就绪信号；当输出寄存器中有新数据字可用时，该线路变为低电平。
    对数据寄存器的读操作完成时，该线路复位为高电平。数据寄存器更新之前，该线路也会变为高电平，以提示此时不应读取器件，
    确保寄存器正在更新时不会发生数据读取操作。

    ADC就绪位。数据写入ADC数据寄存器后此位清0。读取ADC数据寄存器之后，或者在用新转换结
    果更新数据寄存器之前的一定时间内，RDY位自动置1，以告知用户不应读取转换数据。将器件置
    于关断模式或空闲模式时，或者当SYNC变为低电平时，此位也会置1。DOUT/RDY引脚也会指示转
    换何时结束。该引脚可以代替状态寄存器来监视ADC有无转换数据。    
    */
    unsigned long wait_timeout = 0xFFFFF;
    //当获得转换结果时，DOUT/RDY便会变为低电平，表示转换完成
    while(AD7193_RDY_STATE && wait_timeout--)
    {

    }
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
    oldRegValue &= ~(AD7193_CONF_CHAN(0x3FF));                     //清除通道配置位数据
    newRegValue = oldRegValue | AD7193_CONF_CHAN(1 << channel);    //设置新值 
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
    
    ad7193_channel_select(channel);                                //选择通道
    
    oldRegValue = ad7193_get_register_value(AD7193_REG_MODE, 3, 1);//读取模式寄存器数据
    oldRegValue &= ~AD7193_MODE_SEL(0x7);                          //清除模式选择位数据
    newRegValue = oldRegValue | AD7193_MODE_SEL(mode);             //设置新值 
	
   // PMOD1_CS_LOW; 
    ad7193_set_register_value(AD7193_REG_MODE, newRegValue, 3, 0); //将新值重新写入寄存器
    ad7193_wait_ready_go_low();//等待完成
    //PMOD1_CS_HIGH;
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
    oldRegValue &= ~(AD7193_CONF_UNIPOLAR |AD7193_CONF_GAIN(0x7));//清除极性位和增益设置位数据
	
    newRegValue = oldRegValue|(polarity * AD7193_CONF_UNIPOLAR)|AD7193_CONF_GAIN(range); //再重新配置寄存器的值
    ad7193_set_register_value(AD7193_REG_CONF, newRegValue, 3, 1);//写入配置寄存器内

    currentPolarity = polarity;//记录当前设置的极性
    currentGain = 1 << range;  //记录当前设置的增益
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
/*
    单次转换模式下，AD7193在完成转换后处于关断模式。将模式寄存器中的MD2、MD1和MD0分别设置为0、0、1，
    便可启动单次转换，此时AD7193将上电，执行单次转换，然后返回关断模式。片内振荡器上电需要大约1 ms。
    ------------------------------------------------------------------------------------

    DOUT/RDY变为低电平表示转换完成。从数据寄存器中读取数据字后，DOUT/RDY变为高电平。
    如果CS为低电平，DOUT/RDY将保持高电平，直到又一次启动并完成转换为止。
    如果需要，即使DOUT/RDY已变为高电平，也可以多次读取数据寄存器。
    如果使能了多个通道，ADC将依次选择各使能通道，并在该通道上执行转换。
    开始转换后，DOUT/RDY变为高电平并保持该状态，直到获得有效转换结果为止。
    一旦获得转换结果，DOUT/RDY便会变为低电平。然后，ADC选择下一个通道并开始转换。
    在执行下一转换过程中，用户可以读取当前的转换结果。下一转换完成后，数据寄存器便会更新；
    因此，用户读取转换结果的时间有限。ADC在各选择通道上均完成一次转换后，便会返回关断模式。
    如果模式寄存器中的DAT_STA位设置为1，则每次执行数据读取时，状态寄存器的内容将与转换结果一同输出。
    状态寄存器的四个LSB表示对应的转换通道。
*/	
    #if USE_EXT_CLOCK
	    //配置单次转换/使用外部晶振/滤波器输出数据速率
			command = AD7193_MODE_SEL(AD7193_MODE_SINGLE)|\
                      AD7193_MODE_CLKSRC(AD7193_CLK_EXT_MCLK1_2)|\
                      AD7193_MODE_RATE(0x060);		
	  #else
	   //配置单次转换/使用内部晶振/滤波器输出数据速率
			command = AD7193_MODE_SEL(AD7193_MODE_SINGLE)|\
                      AD7193_MODE_CLKSRC(AD7193_CLK_INT)|\
                      AD7193_MODE_RATE(0x060); 
	  #endif

    //PMOD1_CS_LOW;
    ad7193_set_register_value(AD7193_REG_MODE, command, 3, 0); // CS is not modified.
    ad7193_wait_ready_go_low();//等待数据完成
    regData = ad7193_get_register_value(AD7193_REG_DATA, 3, 0);
   // PMOD1_CS_HIGH;
    
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
 /*
    连续转换模式是上电后的默认转换模式。AD7193连续进行转换，
    每次完成转换后，状态寄存器中的RDY位变为低电平。
    如果CS为低电平，则完成一次转换时，DOUT/RDY线路也会变为低电平。
    若要读取转换结果，用户需要写入通信寄存器，指示下一操作为读取数据寄存器。
    从数据寄存器中读取数据字后，DOUT/RDY变为高电平。如需要，用户可以多次读取该寄存器。
    但是，用户必须确保在下一转换完成时，不要对数据寄存器进行访问，否则，新的转换结果将丢失。
    --------------------------------------------------------------------------------

    如果使能了多个通道，ADC将连续循环选择各使能通道，
    每次循环均会在每个通道上执行一次转换。一旦获得转换结果，就会立即更新数据寄存器。
    每次获得转换结果时，DOUT/RDY引脚均会变为低电平。然后，用户可以读取转换结果，同时ADC在下一个使能通道上执行转换。
    如果模式寄存器中的DAT_STA位设置为1，则每次执行数据读取时，状态寄存器的内容将与转换结果一同输出。状态寄存器指示对应的转换通道。

    -------------------------------------------------------------------------------------------------------------------
    在连续转换模式下，ADC按顺序选择各使能通道，然后在该通道上执行转换。当各通道可提供有效转换结果时，DOUT/RDY引脚会给出提示。
    使能多个通道时，状态寄存器的内容应附加到该24位字上，以便用户识别各转换对应的通道。状态寄存器的四个LSB表示对应的转换通道。
    表23和表24显示差分模式和伪差分模式下的通道选项，以及状态寄存器中对应的通道ID值。为了将状态寄存器值附加于转换结果，应将模式寄存器中的DAT_STA位设置为1。
    如果使能多个通道，则每次切换通道时，ADC会给滤波器留出完整的建立时间，以便产生有效转换结果。

    AD7193将通过以下序列自动处理这种状况：
    1. 选择某个通道时，调制器和滤波器将复位。
    2. AD7193允许完整的建立时间以产生有效转换结果。
    3. DOUT/RDY会在有效转换结果可用时给出提示。
    4. AD7193选择下一个使能通道，并在该通道上执行转换。
    5. 当ADC在下一个通道上执行转换时，用户可以读取数据寄存器。 

 */       
    #if USE_EXT_CLOCK
		  //配置连续转换/使用外部晶振/滤波器输出数据速率
			command = AD7193_MODE_SEL(AD7193_MODE_CONT)|\
                      AD7193_MODE_CLKSRC(AD7193_CLK_EXT_MCLK1_2)|\
                      AD7193_MODE_RATE(0x060);		
	  #else
	    //配置连续转换/使用内部晶振/滤波器输出数据速率
			command = AD7193_MODE_SEL(AD7193_MODE_CONT)|\
                      AD7193_MODE_CLKSRC(AD7193_CLK_INT)|\
                      AD7193_MODE_RATE(0x060); 
	  #endif
	
    //PMOD1_CS_LOW;
	  //配置模式寄存器数据
    ad7193_set_register_value(AD7193_REG_MODE, command, 3, 0); //

    for(count = 0; count < sampleNumber; count++)
    {
        ad7193_wait_ready_go_low();//等待完成
        //获取数据寄存器的数据
        samplesAverage += ad7193_get_register_value(AD7193_REG_DATA, 3, 0); // 
    }
    //PMOD1_CS_HIGH;
    samplesAverage = samplesAverage / sampleNumber;//求均值
    
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
 /*
    理论上，使用温度传感器并选择双极性模式时，如果温度为0 K(开尔文)，器件应返回0x800000码。
    为使传感器发挥最佳性能，需要执行单点校准。因此，应记录25°C时的转换结果并计算灵敏度。
    灵敏度约为2815码/°C。温度传感器的计算公式为：
    -- 温度 (K) = (转换结果 − 0x800000)/2815K 
    -- 温度(°C) = 温度(K) − 273
    单点校准之后，内部温度传感器的精度典型值为±2℃。
 */   
    dataReg = ad7193_single_conversion();//执行单次转换
    dataReg = dataReg -0x800000;
    temperature = dataReg / 2815;    // 开氏温度
    temperature = temperature -273;  // 摄氏温度
    
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

/*
    当ADC配置为单极性工作模式时：输出码 = (2^N × AIN × 增益)/VREF
                            
    当ADC配置为双极性工作模式时：输出码 = 2^(N – 1) × [(AIN × 增益/VREF) + 1]
                            
    其中：
        AIN为模拟输入电压。
        增益为PGA设置(1至128)。
        N = 24。	
*/    

    if(currentPolarity == 0 )   // 双极性模式
    {
        voltage = (((float)rawData / (1ul << 23)) - 1) * vRef / currentGain;
    }
    else                       //单极性模式
    {
        voltage = ((float)rawData * vRef) / (1ul << 24) / currentGain;
    }
    
    return voltage;
}

/*
**
*/
void ad7193_bpdsw_set(unsigned char set_val)
{
    unsigned long oldRegValue = 0x0;
    unsigned long newRegValue = 0x0;
    
    oldRegValue = ad7193_get_register_value(AD7193_REG_GPOCON,3, 1);//先读取配置寄存器的值
    oldRegValue &= ~(AD7193_GPOCON_BPDSW);//清除极性位和增益设置位数据

    if(set_val)
    {
      newRegValue = oldRegValue|AD7193_GPOCON_BPDSW; //再重新配置寄存器的值
    }
    ad7193_set_register_value(AD7193_REG_GPOCON, newRegValue, 3, 1);//写入配置寄存器内

}
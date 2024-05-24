#include "INA219_3.h"
#include "delay.h"
u8  INA219_3_busVolt_LSB_mV = 4;   // Bus Voltage LSB value = 4mV
u8  INA219_3_shuntVolt_LSB_uV = 10;  // Shunt Voltage LSB value = 10uV
unsigned short INA219_3_calValue = 0;
 
u32 INA219_3_current_LSB_uA;
u32 INA219_3_power_LSB_mW;
 
INA219_3_DATA INA219_3_data;
 
u8 ram_for_INA219_3[60];
u8 INA219_3process_flag;
#define Open 1
#define Close 0
 
void INA3_SCL_OUT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
 
    RCC_APB2PeriphClockCmd(INA219_3_I2C_GPIO_CLOCK, ENABLE);
    
    /* Configure I2C1 pins: PB12->SCL->OUT */
    GPIO_InitStructure.GPIO_Pin = INA219_3_I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(INA219_3_I2C_PORT, &GPIO_InitStructure);
    GPIO_SetBits(INA219_3_I2C_PORT, INA219_3_I2C_SCL_PIN);
	 
}
 
void INA3_SDA_OUT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
 
    RCC_APB2PeriphClockCmd(INA219_3_I2C_GPIO_CLOCK, ENABLE);
    
    /* Configure I2C1 pins: PB14->SDA-OUT */
    GPIO_InitStructure.GPIO_Pin = INA219_3_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(INA219_3_I2C_PORT, &GPIO_InitStructure);
    GPIO_SetBits(INA219_3_I2C_PORT, INA219_3_I2C_SDA_PIN);
	    

}
 
void INA3_SDA_IN(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
 
    RCC_APB2PeriphClockCmd(INA219_3_I2C_GPIO_CLOCK, ENABLE);
    
    /* Configure I2C1 pins: PB14->SDA-IN */
    GPIO_InitStructure.GPIO_Pin = INA219_3_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(INA219_3_I2C_PORT, &GPIO_InitStructure);
	
}
 
void INA3_IIC_Start(void)
{
  INA3_SDA_OUT();
  INA3_SCL_OUT();
  
  INA3_SDA_SET;
  INA3_SCL_SET;
  INA3_SDA_CLR;
  INA3_SCL_CLR;

}
 
void INA3_IIC_Stop(void)
{
    INA3_SDA_OUT();
 
    INA3_SDA_CLR;
    INA3_SCL_SET;
    INA3_SDA_SET;
	
}
 
void INA3_IIC_Set_Ack(unsigned char ack)
{
    INA3_SDA_OUT();
    
    if(ack)
    {
      INA3_SDA_SET;

    }
    else
    {
      INA3_SDA_CLR;

    }
    
    INA3_SCL_SET;
    INA3_SCL_CLR;

}
 
unsigned char INA3_IIC_Get_Ack(void)
{
    unsigned char ack;
 
    INA3_SDA_IN();
    INA3_SDA_SET;
    INA3_SCL_SET;
    if(INA3_SDA_TST)
    {
      ack = 1;
    }
    else
    {
      ack = 0;
    }
    
    INA3_SCL_CLR;
 
    return(ack);
}
 
void INA3_IIC_Write_8bits(unsigned char dat)
{
  unsigned char i;
  
  INA3_SDA_OUT();
  for(i = 8; i; i--)
  {
    if(dat & 0x80)
    {
      INA3_SDA_SET;
    }
    else
    {
      INA3_SDA_CLR;
    }
    
    INA3_SCL_SET;
    dat <<= 1;
    INA3_SCL_CLR;
  }
}
 
unsigned char INA3_IIC_Read_8bits(void)
{
    unsigned char i, dat;
 
    INA3_SDA_IN();
    INA3_SDA_SET;
    dat = 0;
    for(i = 8; i; i--)
    {
        INA3_SCL_SET;
        dat <<= 1;
        if(INA3_SDA_TST)
          dat++;
        INA3_SCL_CLR;
    }
    
    return(dat);
}
 
void INA3_IIC_Write_Byte(unsigned char reg, unsigned char dat)
{
  unsigned char dev = INA219_3_I2C_ADDRESS;
  
  INA3_IIC_Start();
  
  //  dev &= ~0x01;
  INA3_IIC_Write_8bits(dev);
  INA3_IIC_Get_Ack();
  
  INA3_IIC_Write_8bits(reg);
  INA3_IIC_Get_Ack();
  
  INA3_IIC_Write_8bits(dat);
  INA3_IIC_Get_Ack();
  
  INA3_IIC_Stop();
}
 
unsigned char INA3_IIC_Read_Byte(unsigned char reg)
{
  unsigned char dat;
  unsigned char dev = INA219_3_I2C_ADDRESS;
  
  INA3_IIC_Start();
  
  //  dev &= ~0x01;
  INA3_IIC_Write_8bits(dev);
  INA3_IIC_Get_Ack();
  
  INA3_IIC_Write_8bits(reg);
  INA3_IIC_Get_Ack();
  
  INA3_IIC_Start();
  
  dev |= 0x01;
  INA3_IIC_Write_8bits(dev);
  INA3_IIC_Get_Ack();
  
  dat = INA3_IIC_Read_8bits();
  INA3_IIC_Set_Ack(1);
  
  INA3_IIC_Stop();
  
  return (dat);
}
 
void INA3_IIC_Write_Bytes(unsigned char reg, unsigned char *dat, unsigned char num)
{
  unsigned char dev = INA219_3_I2C_ADDRESS;
  
  INA3_IIC_Start();
  
  //  dev &= ~0x01;
  INA3_IIC_Write_8bits(dev);
  INA3_IIC_Get_Ack();
  
  INA3_IIC_Write_8bits(reg);
  INA3_IIC_Get_Ack();
  
  while(num--)
  {
    INA3_IIC_Write_8bits(*dat);
    INA3_IIC_Get_Ack();
    dat++;
  }
  
  INA3_IIC_Stop();
}
 
void INA3_IIC_Read_Bytes(unsigned char reg, unsigned char *dat, unsigned char num)
{
  unsigned char *tmp = dat;
  unsigned char dev = INA219_3_I2C_ADDRESS;
  
  INA3_IIC_Start();
  
  //  dev &= ~0x01;
  INA3_IIC_Write_8bits(dev);
  INA3_IIC_Get_Ack();
  
  INA3_IIC_Write_8bits(reg);
  INA3_IIC_Get_Ack();
  
  INA3_IIC_Start();
  
  dev |= 0x01;
  INA3_IIC_Write_8bits(dev);
  INA3_IIC_Get_Ack();
  
  while(num--)
  {
    *tmp = INA3_IIC_Read_8bits();
    if(num == 0)
      INA3_IIC_Set_Ack(1);
    else
      INA3_IIC_Set_Ack(0);
    tmp++;
  }
  
  INA3_IIC_Stop();
}
 
void INA219_3_Write_Register(unsigned char reg, unsigned int dat)
{
    unsigned char val[2];
    
    val[0] = (unsigned char)(dat >> 8);
    val[1] = (unsigned char)(dat & 0xFF);
    INA3_IIC_Write_Bytes(reg, val, 2);
}
 
void INA219_3_Read_Register(unsigned char reg, signed short *dat)
{
    //printf("read reg == %d\r\n",reg);
  unsigned char val[2];
  
  INA3_IIC_Read_Bytes(reg, val, 2);
  *dat = ((unsigned int)(val[0]) << 8) + val[1];
  
    //printf("data1 == %x\r\n",val[0]);
    //printf("data2 == %x\r\n",val[1]);
    
}
 
// INA219_3 Set Calibration 16V/16A(Max) 0.02|?
void INA219_3_SetCalibration_16V_16A(void)
{
  u16 configValue;
  
  // By default we use a pretty huge range for the input voltage,
  // which probably isn't the most appropriate choice for system
  // that don't use a lot of power.  But all of the calculations
  // are shown below if you want to change the settings.  You will
  // also need to change any relevant register settings, such as
  // setting the VBUS_MAX to 16V instead of 32V, etc.
  
  // VBUS_MAX     = 16V   (Assumes 16V, can also be set to 32V)
  // VSHUNT_MAX   = 0.32  (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
  // RSHUNT       = 0.02   (Resistor value in ohms)
  
  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 16A
  
  // 2. Determine max expected current
  // MaxExpected_I = 16A
  
  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.00048            (0.48mA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0,00390            (3.9mA per bit)
  
  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.00050            (500uA per bit)
  
  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 4096 (0x1000)
  
  INA219_3_calValue = 0x0D90;  //0x1000;
  INA219_3_calValue = 0x1000*358/362;  //0x1000;
	
	
  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.01 (10mW per bit)
  
  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 16.3835A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.32V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If
  
  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 1.6 * 16V
  // MaximumPower = 256W
  
  // Set multipliers to convert raw current/power values
  INA219_3_current_LSB_uA = 100;     // Current LSB = 500uA per bit
  INA219_3_power_LSB_mW = 2;        // Power LSB = 10mW per bit = 20 * Current LSB
  
  // Set Calibration register to 'Cal' calculated above
  INA219_3_Write_Register(INA219_3_REG_CALIBRATION, INA219_3_calValue);
  
  // Set Config register to take into account the settings above
  configValue = ( INA219_3_CFG_BVOLT_RANGE_16V | INA219_3_CFG_SVOLT_RANGE_320MV | INA219_3_CFG_BADCRES_12BIT_16S_8MS | INA219_3_CFG_SADCRES_12BIT_16S_8MS | INA219_3_CFG_MODE_SANDBVOLT_CONTINUOUS );
  //configValue = ( INA219_3_CFG_BVOLT_RANGE_32V | INA219_3_CFG_SVOLT_RANGE_320MV | INA219_3_CFG_BADCRES_12BIT_128S_68MS | INA219_3_CFG_SADCRES_12BIT_128S_68MS | INA219_3_CFG_MODE_SANDBVOLT_CONTINUOUS );
  
	
	
  INA219_3_Write_Register(INA219_3_REG_CONFIG, configValue);
}
 
void INA219_3_configureRegisters(void)
{
  delay_ms(15);
  
  INA219_3_SetCalibration_16V_16A();
}
 
void INA219_3_gpio_init(void)
{
    INA3_SCL_OUT();
    INA3_SDA_OUT();
}
 
void INA219_3_init(void)
{
  INA219_3_gpio_init();
  
  INA219_3_configureRegisters();
}


signed short INA219_3_GetBusVoltage_raw(void)
{
  signed short val;
  
  INA219_3_Read_Register(INA219_3_REG_BUSVOLTAGE, &val);
  val >>= 3;                      // Shift to the right 3 to drop CNVR and OVF
  
  return (val);
}
 
signed short INA219_3_GetCurrent_raw(void)
{
  signed short val;
  
  // Sometimes a sharp load will reset the INA219_3, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  INA219_3_Write_Register(INA219_3_REG_CALIBRATION, INA219_3_calValue);
  
  // Now we can safely read the CURRENT register!
  INA219_3_Read_Register(INA219_3_REG_CURRENT, &val);
  
  return (val);
}
 
 
signed short INA219_3_GetBusVoltage_mV(void)
{
  signed short val;
  
  INA219_3_Read_Register(INA219_3_REG_BUSVOLTAGE, &val);
  val >>= 3;                      // Shift to the right 3 to drop CNVR and OVF
  val *= INA219_3_busVolt_LSB_mV;   // multiply by LSB(4mV)
  
  return (val);
}
 
s32 INA219_3_GetShuntVoltage_uV(void)
{
  s32 val;
  s16 reg;
  
  INA219_3_Read_Register(INA219_3_REG_SHUNTVOLTAGE, &reg);
  val = (s32)reg * INA219_3_shuntVolt_LSB_uV;   // multiply by LSB(10uV)
  
  return (val);
}
 
s32 INA219_3_GetCurrent_uA(void)
{
  s32 val;
  s16 reg;
  
  // Sometimes a sharp load will reset the INA219_3, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  INA219_3_Write_Register(INA219_3_REG_CALIBRATION, INA219_3_calValue);
  
  // Now we can safely read the CURRENT register!
  INA219_3_Read_Register(INA219_3_REG_CURRENT, &reg);
  
  val = (s32)reg * INA219_3_current_LSB_uA;
  
  return (val);
}
 
s32 INA219_3_GetPower_mW(void)
{
  s32 val;
  s16 reg;
  
  // Sometimes a sharp load will reset the INA219_3, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  INA219_3_Write_Register(INA219_3_REG_CALIBRATION, INA219_3_calValue);
  
  // Now we can safely read the POWER register!
  INA219_3_Read_Register(INA219_3_REG_POWER, &reg);
  
  val = (s32)reg * INA219_3_power_LSB_mW;
  
  return (val);
}
 
void INA3_Process(void)
{
    if(1)
    {
        //INA219_3process_flag = Close;
        
        INA219_3_data.voltage_INA219_3 = INA219_3_GetBusVoltage_mV();
      //  printf("voltage_INA219_3 is %d\r\n",INA219_3_data.voltage_INA219_3);
        
        INA219_3_data.shunt_INA219_3 = INA219_3_GetShuntVoltage_uV();
       // printf("shunt_INA219_3 is %ld\r\n",INA219_3_data.shunt_INA219_3);
            
        INA219_3_data.current_INA219_3 = INA219_3_GetCurrent_uA();
       // printf("current_INA219_3 is %ld\r\n",INA219_3_data.current_INA219_3);
        
        INA219_3_data.power_INA219_3 = INA219_3_GetPower_mW();
      //  printf("power_INA219_3 is %ld\r\n",INA219_3_data.power_INA219_3);
				
    }
}
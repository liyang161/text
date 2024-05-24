#include "ina226.h"
#include "delay.h"
u8  ina226_busVolt_LSB_mV = 4;   // Bus Voltage LSB value = 4mV
u8  ina226_shuntVolt_LSB_uV = 10;  // Shunt Voltage LSB value = 10uV
unsigned short ina226_calValue = 0;
 
u32 ina226_current_LSB_uA;
u32 ina226_power_LSB_mW;
 
INA226_DATA ina226_data;
 
u8 ram_for_ina226[60];
u8 INA226process_flag;
#define Open 1
#define Close 0
 
void INA226_SCL_OUT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
 
    RCC_APB2PeriphClockCmd(INA226_I2C_GPIO_CLOCK, ENABLE);
    
    /* Configure I2C1 pins: PB12->SCL->OUT */
    GPIO_InitStructure.GPIO_Pin = INA226_I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(INA226_I2C_PORT, &GPIO_InitStructure);
    GPIO_SetBits(INA226_I2C_PORT, INA226_I2C_SCL_PIN);
}
 
void INA226_SDA_OUT(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
 
    RCC_APB2PeriphClockCmd(INA226_I2C_GPIO_CLOCK, ENABLE);
    
    /* Configure I2C1 pins: PB14->SDA-OUT */
    GPIO_InitStructure.GPIO_Pin = INA226_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(INA226_I2C_PORT, &GPIO_InitStructure);
    GPIO_SetBits(INA226_I2C_PORT, INA226_I2C_SDA_PIN);
}
 
void INA226_SDA_IN(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
 
    RCC_APB2PeriphClockCmd(INA226_I2C_GPIO_CLOCK, ENABLE);
    
    /* Configure I2C1 pins: PB14->SDA-IN */
    GPIO_InitStructure.GPIO_Pin = INA226_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(INA226_I2C_PORT, &GPIO_InitStructure);
}
 
void INA226_IIC_Start(void)
{
  INA226_SDA_OUT();
  INA226_SCL_OUT();
  
  INA226_SDA_SET;
  INA226_SCL_SET;
  INA226_SDA_CLR;
  INA226_SCL_CLR;
}
 
void INA226_IIC_Stop(void)
{
    INA226_SDA_OUT();
 
    INA226_SDA_CLR;
    INA226_SCL_SET;
    INA226_SDA_SET;
}
 
void INA226_IIC_Set_Ack(unsigned char ack)
{
    INA226_SDA_OUT();
    
    if(ack)
    {
      INA226_SDA_SET;
    }
    else
    {
      INA226_SDA_CLR;
    }
    
    INA226_SCL_SET;
    INA226_SCL_CLR;
}
 
unsigned char INA226_IIC_Get_Ack(void)
{
    unsigned char ack;
 
    INA226_SDA_IN();
    INA226_SDA_SET;
    INA226_SCL_SET;
    if(INA226_SDA_TST)
    {
      ack = 1;
    }
    else
    {
      ack = 0;
    }
    
    INA226_SCL_CLR;
 
    return(ack);
}
 
void INA226_IIC_Write_8bits(unsigned char dat)
{
  unsigned char i;
  
  INA226_SDA_OUT();
  for(i = 8; i; i--)
  {
    if(dat & 0x80)
    {
      INA226_SDA_SET;
    }
    else
    {
      INA226_SDA_CLR;
    }
    
    INA226_SCL_SET;
    dat <<= 1;
    INA226_SCL_CLR;
  }
}
 
unsigned char INA226_IIC_Read_8bits(void)
{
    unsigned char i, dat;
 
    INA226_SDA_IN();
    INA226_SDA_SET;
    dat = 0;
    for(i = 8; i; i--)
    {
        INA226_SCL_SET;
        dat <<= 1;
        if(INA226_SDA_TST)
          dat++;
        INA226_SCL_CLR;
    }
    
    return(dat);
}
 
void INA226_IIC_Write_Byte(unsigned char reg, unsigned char dat)
{
  unsigned char dev = INA226_I2C_ADDRESS;
  
  INA226_IIC_Start();
  
  //  dev &= ~0x01;
  INA226_IIC_Write_8bits(dev);
  INA226_IIC_Get_Ack();
  
  INA226_IIC_Write_8bits(reg);
  INA226_IIC_Get_Ack();
  
  INA226_IIC_Write_8bits(dat);
  INA226_IIC_Get_Ack();
  
  INA226_IIC_Stop();
}
 
unsigned char INA226_IIC_Read_Byte(unsigned char reg)
{
  unsigned char dat;
  unsigned char dev = INA226_I2C_ADDRESS;
  
  INA226_IIC_Start();
  
  //  dev &= ~0x01;
  INA226_IIC_Write_8bits(dev);
  INA226_IIC_Get_Ack();
  
  INA226_IIC_Write_8bits(reg);
  INA226_IIC_Get_Ack();
  
  INA226_IIC_Start();
  
  dev |= 0x01;
  INA226_IIC_Write_8bits(dev);
  INA226_IIC_Get_Ack();
  
  dat = INA226_IIC_Read_8bits();
  INA226_IIC_Set_Ack(1);
  
  INA226_IIC_Stop();
  
  return (dat);
}
 
void INA226_IIC_Write_Bytes(unsigned char reg, unsigned char *dat, unsigned char num)
{
  unsigned char dev = INA226_I2C_ADDRESS;
  
  INA226_IIC_Start();
  
  //  dev &= ~0x01;
  INA226_IIC_Write_8bits(dev);
  INA226_IIC_Get_Ack();
  
  INA226_IIC_Write_8bits(reg);
  INA226_IIC_Get_Ack();
  
  while(num--)
  {
    INA226_IIC_Write_8bits(*dat);
    INA226_IIC_Get_Ack();
    dat++;
  }
  
  INA226_IIC_Stop();
}
 
void INA226_IIC_Read_Bytes(unsigned char reg, unsigned char *dat, unsigned char num)
{
  unsigned char *tmp = dat;
  unsigned char dev = INA226_I2C_ADDRESS;
  
  INA226_IIC_Start();
  
  //  dev &= ~0x01;
  INA226_IIC_Write_8bits(dev);
  INA226_IIC_Get_Ack();
  
  INA226_IIC_Write_8bits(reg);
  INA226_IIC_Get_Ack();
  
  INA226_IIC_Start();
  
  dev |= 0x01;
  INA226_IIC_Write_8bits(dev);
  INA226_IIC_Get_Ack();
  
  while(num--)
  {
    *tmp = INA226_IIC_Read_8bits();
    if(num == 0)
      INA226_IIC_Set_Ack(1);
    else
      INA226_IIC_Set_Ack(0);
    tmp++;
  }
  
  INA226_IIC_Stop();
}
 
void ina226_Write_Register(unsigned char reg, unsigned int dat)
{
    unsigned char val[2];
    
    val[0] = (unsigned char)(dat >> 8);
    val[1] = (unsigned char)(dat & 0xFF);
    INA226_IIC_Write_Bytes(reg, val, 2);
}
 
void ina226_Read_Register(unsigned char reg, signed short *dat)
{
    //printf("read reg == %d\r\n",reg);
  unsigned char val[2];
  
  INA226_IIC_Read_Bytes(reg, val, 2);
  *dat = ((unsigned int)(val[0]) << 8) + val[1];
  
    //printf("data1 == %x\r\n",val[0]);
    //printf("data2 == %x\r\n",val[1]);
    
}
 
// INA226 Set Calibration 16V/16A(Max) 0.02|?
void ina226_SetCalibration_16V_16A(void)
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
  
  ina226_calValue = 0x0D90;  //0x1000;
  ina226_calValue = 0x1000;  //0x1000;
	ina226_calValue = 0x0A00;
	ina226_calValue = 0x0A00/4.89;
	//ina226_calValue =  15000*1000/(1<<15)*2;
	
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
  ina226_current_LSB_uA = 100;     // Current LSB = 500uA per bit
  ina226_power_LSB_mW = 2;        // Power LSB = 10mW per bit = 20 * Current LSB
  
  // Set Calibration register to 'Cal' calculated above
  ina226_Write_Register(INA226_REG_CALIBRATION, ina226_calValue);
  
  // Set Config register to take into account the settings above
  //configValue = ( INA226_CFG_BVOLT_RANGE_16V | INA226_CFG_SVOLT_RANGE_320MV | INA226_CFG_BADCRES_12BIT_16S_8MS | INA226_CFG_SADCRES_12BIT_16S_8MS | INA226_CFG_MODE_SANDBVOLT_CONTINUOUS );
  configValue = ( INA226_CFG_BVOLT_RANGE_32V | INA226_CFG_SVOLT_RANGE_320MV | INA226_CFG_BADCRES_12BIT_128S_68MS | INA226_CFG_SADCRES_12BIT_128S_68MS | INA226_CFG_MODE_SANDBVOLT_CONTINUOUS );
  
	
	
  //ina226_Write_Register(INA226_REG_CONFIG, configValue);
	ina226_Write_Register(INA226_REG_CONFIG, 0x4527);
}
 
void ina226_configureRegisters(void)
{
  delay_ms(15);
  
  ina226_SetCalibration_16V_16A();
}
 
void ina226_gpio_init(void)
{
    INA226_SCL_OUT();
    INA226_SDA_OUT();
}
 
void ina226_init(void)
{
  ina226_gpio_init();
  
  ina226_configureRegisters();
}


signed short ina226_GetBusVoltage_raw(void)
{
  signed short val;
  
  ina226_Read_Register(INA226_REG_BUSVOLTAGE, &val);
  val >>= 3;                      // Shift to the right 3 to drop CNVR and OVF
  
  return (val);
}
 
signed short ina226_GetCurrent_raw(void)
{
  signed short val;
  
  // Sometimes a sharp load will reset the INA226, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  ina226_Write_Register(INA226_REG_CALIBRATION, ina226_calValue);
  
  // Now we can safely read the CURRENT register!
  ina226_Read_Register(INA226_REG_CURRENT, &val);
  
  return (val);
}
 
 
signed short ina226_GetBusVoltage_mV(void)
{
  signed short val;
  
  ina226_Read_Register(INA226_REG_BUSVOLTAGE, &val);
  val >>= 3;                      // Shift to the right 3 to drop CNVR and OVF
  val *= ina226_busVolt_LSB_mV;   // multiply by LSB(4mV)
  
  return (val);
}
 
s32 ina226_GetShuntVoltage_uV(void)
{
  s32 val;
  s16 reg;
  
  ina226_Read_Register(INA226_REG_SHUNTVOLTAGE, &reg);
  val = (s32)reg * ina226_shuntVolt_LSB_uV;   // multiply by LSB(10uV)
  
  return (val);
}
 
s32 ina226_GetCurrent_uA(void)
{
  s32 val;
  s16 reg;
  
  // Sometimes a sharp load will reset the INA226, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  ina226_Write_Register(INA226_REG_CALIBRATION, ina226_calValue);
  
  // Now we can safely read the CURRENT register!
  ina226_Read_Register(INA226_REG_CURRENT, &reg);
  
  val = (s32)reg * ina226_current_LSB_uA;
  
  return (val);
}
 
s32 ina226_GetPower_mW(void)
{
  s32 val;
  s16 reg;
  
  // Sometimes a sharp load will reset the INA226, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  ina226_Write_Register(INA226_REG_CALIBRATION, ina226_calValue);
  
  // Now we can safely read the POWER register!
  ina226_Read_Register(INA226_REG_POWER, &reg);
  
  val = (s32)reg * ina226_power_LSB_mW;
  
  return (val);
}
 
void INA226_Process(void)
{
    if(1)
    {
        //INA226process_flag = Close;
        
        ina226_data.voltage_ina226 = ina226_GetBusVoltage_mV();
      //  printf("voltage_ina226 is %d\r\n",ina226_data.voltage_ina226);
        
        ina226_data.shunt_ina226 = ina226_GetShuntVoltage_uV();
       // printf("shunt_ina226 is %ld\r\n",ina226_data.shunt_ina226);
            
        ina226_data.current_ina226 = ina226_GetCurrent_uA();
       // printf("current_ina226 is %ld\r\n",ina226_data.current_ina226);
        
        ina226_data.power_ina226 = ina226_GetPower_mW();
      //  printf("power_ina226 is %ld\r\n",ina226_data.power_ina226);
				
    }
}
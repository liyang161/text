#ifndef __INA226_H
#define __INA226_H
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#define INA226_I2C_PORT                        GPIOB
#define INA226_I2C_GPIO_CLOCK            RCC_APB2Periph_GPIOB
#define INA226_I2C_SCL_PIN                GPIO_Pin_11
#define INA226_I2C_SDA_PIN                GPIO_Pin_10
 
#define INA226_SCL_SET     GPIO_SetBits(INA226_I2C_PORT,INA226_I2C_SCL_PIN)
#define INA226_SDA_SET     GPIO_SetBits(INA226_I2C_PORT, INA226_I2C_SDA_PIN)
 
#define INA226_SCL_CLR     GPIO_ResetBits(INA226_I2C_PORT,INA226_I2C_SCL_PIN)
#define INA226_SDA_CLR     GPIO_ResetBits(INA226_I2C_PORT,INA226_I2C_SDA_PIN)
 
#define INA226_SDA_TST     GPIO_ReadInputDataBit(INA226_I2C_PORT,INA226_I2C_SDA_PIN)
 
/*----------------------------------------------------------------------------*/
// I2C Address Options
#define INA226_I2C_ADDRESS_CONF_0               (u8)(0x40 << 1)     // A0 = GND, A1 = GND
#define INA226_I2C_ADDRESS_CONF_1               (u8)(0x41 << 1)     // A0 = VS+, A1 = GND
#define INA226_I2C_ADDRESS_CONF_2               (u8)(0x42 << 1)     // A0 = SDA, A1 = GND
#define INA226_I2C_ADDRESS_CONF_3               (u8)(0x43 << 1)     // A0 = SCL, A1 = GND
#define INA226_I2C_ADDRESS_CONF_4               (u8)(0x44 << 1)     // A0 = GND, A1 = VS+
#define INA226_I2C_ADDRESS_CONF_5               (u8)(0x45 << 1)     // A0 = VS+, A1 = VS+
#define INA226_I2C_ADDRESS_CONF_6               (u8)(0x46 << 1)     // A0 = SDA, A1 = VS+
#define INA226_I2C_ADDRESS_CONF_7               (u8)(0x47 << 1)     // A0 = SCL, A1 = VS+
#define INA226_I2C_ADDRESS_CONF_8               (u8)(0x48 << 1)     // A0 = GND, A1 = SDA
#define INA226_I2C_ADDRESS_CONF_9               (u8)(0x49 << 1)     // A0 = VS+, A1 = SDA
#define INA226_I2C_ADDRESS_CONF_A               (u8)(0x4A << 1)     // A0 = SDA, A1 = SDA
#define INA226_I2C_ADDRESS_CONF_B               (u8)(0x4B << 1)     // A0 = SCL, A1 = SDA
#define INA226_I2C_ADDRESS_CONF_C               (u8)(0x4C << 1)     // A0 = GND, A1 = SCL
#define INA226_I2C_ADDRESS_CONF_D               (u8)(0x4D << 1)     // A0 = VS+, A1 = SCL
#define INA226_I2C_ADDRESS_CONF_E               (u8)(0x4E << 1)     // A0 = SDA, A1 = SCL
#define INA226_I2C_ADDRESS_CONF_F               (u8)(0x4F << 1)     // A0 = SCL, A1 = SCL
#define INA226_I2C_ADDRESS                      INA226_I2C_ADDRESS_CONF_0
 
 
/*----------------------------------------------------------------------------*/
// Register Addresses
#define INA226_REG_CONFIG                       (u8)(0x00)      // CONFIG REGISTER (R/W)
#define INA226_REG_SHUNTVOLTAGE                 (u8)(0x01)      // SHUNT VOLTAGE REGISTER (R)
#define INA226_REG_BUSVOLTAGE                   (u8)(0x02)      // BUS VOLTAGE REGISTER (R)
#define INA226_REG_POWER                        (u8)(0x03)      // POWER REGISTER (R)
#define INA226_REG_CURRENT                      (u8)(0x04)      // CURRENT REGISTER (R)
#define INA226_REG_CALIBRATION                  (u8)(0x05)      // CALIBRATION REGISTER (R/W)
 
 
/*----------------------------------------------------------------------------*/
// Macros for assigning config bits
#define INA226_CFGB_RESET(x)                    (u16)((x & 0x01) << 15)     // Reset Bit
#define INA226_CFGB_BUSV_RANGE(x)               (u16)((x & 0x01) << 13)     // Bus Voltage Range
#define INA226_CFGB_PGA_RANGE(x)                (u16)((x & 0x03) << 11)     // Shunt Voltage Range
#define INA226_CFGB_BADC_RES_AVG(x)             (u16)((x & 0x0F) << 7)      // Bus ADC Resolution/Averaging
#define INA226_CFGB_SADC_RES_AVG(x)             (u16)((x & 0x0F) << 3)      // Shunt ADC Resolution/Averaging
#define INA226_CFGB_MODE(x)                     (u16) (x & 0x07)            // Operating Mode
 
 
/*----------------------------------------------------------------------------*/
// Configuration Register
#define INA226_CFG_RESET                        INA226_CFGB_RESET(1)            // Reset Bit
 
#define INA226_CFG_BVOLT_RANGE_MASK             INA226_CFGB_BUSV_RANGE(1)       // Bus Voltage Range Mask
#define INA226_CFG_BVOLT_RANGE_16V              INA226_CFGB_BUSV_RANGE(0)       // 0-16V Range
#define INA226_CFG_BVOLT_RANGE_32V              INA226_CFGB_BUSV_RANGE(1)       // 0-32V Range (default)
 
#define INA226_CFG_SVOLT_RANGE_MASK             INA226_CFGB_PGA_RANGE(3)        // Shunt Voltage Range Mask
#define INA226_CFG_SVOLT_RANGE_40MV             INA226_CFGB_PGA_RANGE(0)        // Gain 1, 40mV Range
#define INA226_CFG_SVOLT_RANGE_80MV             INA226_CFGB_PGA_RANGE(1)        // Gain 2, 80mV Range
#define INA226_CFG_SVOLT_RANGE_160MV            INA226_CFGB_PGA_RANGE(2)        // Gain 4, 160mV Range
#define INA226_CFG_SVOLT_RANGE_320MV            INA226_CFGB_PGA_RANGE(3)        // Gain 8, 320mV Range (default)
 
#define INA226_CFG_BADCRES_MASK                 INA226_CFGB_BADC_RES_AVG(15)    // Bus ADC Resolution and Averaging Mask
#define INA226_CFG_BADCRES_9BIT_1S_84US         INA226_CFGB_BADC_RES_AVG(0)     // 1 x 9-bit Bus sample
#define INA226_CFG_BADCRES_10BIT_1S_148US       INA226_CFGB_BADC_RES_AVG(1)     // 1 x 10-bit Bus sample
#define INA226_CFG_BADCRES_11BIT_1S_276US       INA226_CFGB_BADC_RES_AVG(2)     // 1 x 11-bit Bus sample
#define INA226_CFG_BADCRES_12BIT_1S_532US       INA226_CFGB_BADC_RES_AVG(3)     // 1 x 12-bit Bus sample (default)
#define INA226_CFG_BADCRES_12BIT_2S_1MS         INA226_CFGB_BADC_RES_AVG(9)     // 2 x 12-bit Bus samples averaged together
#define INA226_CFG_BADCRES_12BIT_4S_2MS         INA226_CFGB_BADC_RES_AVG(10)    // 4 x 12-bit Bus samples averaged together
#define INA226_CFG_BADCRES_12BIT_8S_4MS         INA226_CFGB_BADC_RES_AVG(11)    // 8 x 12-bit Bus samples averaged together
#define INA226_CFG_BADCRES_12BIT_16S_8MS        INA226_CFGB_BADC_RES_AVG(12)    // 16 x 12-bit Bus samples averaged together
#define INA226_CFG_BADCRES_12BIT_32S_17MS       INA226_CFGB_BADC_RES_AVG(13)    // 32 x 12-bit Bus samples averaged together
#define INA226_CFG_BADCRES_12BIT_64S_34MS       INA226_CFGB_BADC_RES_AVG(14)    // 64 x 12-bit Bus samples averaged together
#define INA226_CFG_BADCRES_12BIT_128S_68MS      INA226_CFGB_BADC_RES_AVG(15)    // 128 x 12-bit Bus samples averaged together
 
#define INA226_CFG_SADCRES_MASK                 INA226_CFGB_SADC_RES_AVG(15)    // Shunt ADC Resolution and Averaging Mask
#define INA226_CFG_SADCRES_9BIT_1S_84US         INA226_CFGB_SADC_RES_AVG(0)     // 1 x 9-bit Shunt sample
#define INA226_CFG_SADCRES_10BIT_1S_148US       INA226_CFGB_SADC_RES_AVG(1)     // 1 x 10-bit Shunt sample
#define INA226_CFG_SADCRES_11BIT_1S_276US       INA226_CFGB_SADC_RES_AVG(2)     // 1 x 11-bit Shunt sample
#define INA226_CFG_SADCRES_12BIT_1S_532US       INA226_CFGB_SADC_RES_AVG(3)     // 1 x 12-bit Shunt sample (default)
#define INA226_CFG_SADCRES_12BIT_2S_1MS         INA226_CFGB_SADC_RES_AVG(9)     // 2 x 12-bit Shunt samples averaged together
#define INA226_CFG_SADCRES_12BIT_4S_2MS         INA226_CFGB_SADC_RES_AVG(10)    // 4 x 12-bit Shunt samples averaged together
#define INA226_CFG_SADCRES_12BIT_8S_4MS         INA226_CFGB_SADC_RES_AVG(11)    // 8 x 12-bit Shunt samples averaged together
#define INA226_CFG_SADCRES_12BIT_16S_8MS        INA226_CFGB_SADC_RES_AVG(12)    // 16 x 12-bit Shunt samples averaged together
#define INA226_CFG_SADCRES_12BIT_32S_17MS       INA226_CFGB_SADC_RES_AVG(13)    // 32 x 12-bit Shunt samples averaged together
#define INA226_CFG_SADCRES_12BIT_64S_34MS       INA226_CFGB_SADC_RES_AVG(14)    // 64 x 12-bit Shunt samples averaged together
#define INA226_CFG_SADCRES_12BIT_128S_68MS      INA226_CFGB_SADC_RES_AVG(15)    // 128 x 12-bit Shunt samples averaged together
 
#define INA226_CFG_MODE_MASK                    INA226_CFGB_MODE(7)             // Operating Mode Mask
#define INA226_CFG_MODE_POWERDOWN               INA226_CFGB_MODE(0)             // Power-Down
#define INA226_CFG_MODE_SVOLT_TRIGGERED         INA226_CFGB_MODE(1)             // Shunt Voltage, Triggered
#define INA226_CFG_MODE_BVOLT_TRIGGERED         INA226_CFGB_MODE(2)             // Bus Voltage, Triggered
#define INA226_CFG_MODE_SANDBVOLT_TRIGGERED     INA226_CFGB_MODE(3)             // Shunt and Bus, Triggered
#define INA226_CFG_MODE_ADCOFF                  INA226_CFGB_MODE(4)             // ADC Off (disabled)
#define INA226_CFG_MODE_SVOLT_CONTINUOUS        INA226_CFGB_MODE(5)             // Shunt Voltage, Continuous
#define INA226_CFG_MODE_BVOLT_CONTINUOUS        INA226_CFGB_MODE(6)             // Bus Voltage, Continuous
#define INA226_CFG_MODE_SANDBVOLT_CONTINUOUS    INA226_CFGB_MODE(7)             // Shunt and Bus, Continuous (default)
 
 
/*----------------------------------------------------------------------------*/
// Bus Voltage Register
#define INA226_BVOLT_CNVR                       (u16)(0x0002)       // Conversion Ready
#define INA226_BVOLT_OVF                        (u16)(0x0001)       // Math Overflow Flag
 
typedef struct
{
  signed short voltage_ina226;
  signed long shunt_ina226;
  signed long current_ina226;
  signed long power_ina226;
}INA226_DATA;
 
 
extern u8  ina226_busVolt_LSB_mV;
extern u8  ina226_shuntVolt_LSB_uV;
extern unsigned short ina226_calValue;
 
extern u32 ina226_current_LSB_uA;
extern u32 ina226_power_LSB_mW;
 
extern void ina226_init(void);
extern void INA226_Process(void);
extern signed short ina226_GetBusVoltage_raw(void);
extern signed short ina226_GetCurrent_raw(void);
extern signed short ina226_GetBusVoltage_mV(void);
extern s32 ina226_GetShuntVoltage_uV(void);
extern s32 ina226_GetCurrent_uA(void);
extern s32 ina226_GetPower_mW(void);
 
#endif
/*
 * @Author: xmprocat
 * @Date: 2022-07-31 15:35:40
 * @LastEditors: xmprocat
 * @LastEditTime: 2022-08-06 21:23:35
 * @Description: ina3221_2 模拟IIC驱动
 参考:  https://blog.csdn.net/u011493046/article/details/126202237
 */

#include "INA3221_2.h"
#include "delay.h"
#include <stdlib.h>
/*****************硬件连接定义*********************/
#define INA3221_2_SCL_PORT GPIOB
#define INA3221_2_SCL_PIN GPIO_Pin_7
#define INA219_I2C_GPIO_CLOCK            RCC_APB2Periph_GPIOB
#define INA3221_2_SDA_PORT GPIOB
#define INA3221_2_SDA_PIN GPIO_Pin_6

/************************延时接口****************************/
#define delay_nms_3221_2 delay_ms       //毫秒延时
// #define delay_nns_3221_2 delay_nus_3221_2       //毫秒延时

#define INA3221_2_CFG_REG         0x00    //配置寄存器
#define INA3221_2_CH1SHUNT_REG    0x01    //通道 1 分流电压
#define INA3221_2_CH1BUS_REG      0x02    //通道 1 总线电压
#define INA3221_2_CH2SHUNT_REG    0x03    //通道 2 分流电压
#define INA3221_2_CH2BUS_REG      0x04    //通道 2 总线电压
#define INA3221_2_CH3SHUNT_REG    0x05    //通道 3 分流电压
#define INA3221_2_CH3BUS_REG      0x06    //通道 3 总线电压
#define INA3221_2_CH1CAL_REG      0x07    //通道 1 严重警报限制
#define INA3221_2_CH1WAL_REG      0x08    //通道 1 警告警报限制
#define INA3221_2_CH2CAL_REG      0x09    //通道 2 严重警报限制
#define INA3221_2_CH2WAL_REG      0x0A    //通道 2 警告警报限制
#define INA3221_2_CH3CAL_REG      0x0B    //通道 3 严重警报限制
#define INA3221_2_CH3WAL_REG      0x0C    //通道 3 警告警报限制
#define INA3221_2_SVS_REG         0x0D    //分流电压和
#define INA3221_2_SVSLIMIT_REG    0x0E    //分流电压和限制
#define INA3221_2_ME_REG          0x0F    //屏蔽/启用 警报
#define INA3221_2_PVUPPER_REG     0x10    //功率有效上限
#define INA3221_2_PVLOW_REG       0x11    //功率有效下限
#define INA3221_2_MANUID_REG      0xFE    //制造商标识号
#define INA3221_2_DIEID_REG       0xFF    //模具标识号

#define INA3221_2_MANU_ID     0x5449  //唯一制造商标识号
#define INA3221_2_DIE_ID      0x3220  //唯一模具标识号


#define IIC_SCL_L_3221_2 GPIO_ResetBits(INA3221_2_SCL_PORT, INA3221_2_SCL_PIN)
#define IIC_SCL_H_3221_2 GPIO_SetBits(INA3221_2_SCL_PORT, INA3221_2_SCL_PIN)

#define IIC_SDA_L_3221_2 GPIO_ResetBits(INA3221_2_SDA_PORT, INA3221_2_SDA_PIN)
#define IIC_SDA_H_3221_2 GPIO_SetBits(INA3221_2_SDA_PORT, INA3221_2_SDA_PIN)
#define SDA_READ_3221_2 GPIO_ReadInputDataBit(INA3221_2_SDA_PORT, INA3221_2_SDA_PIN)


static void delay_nns_3221_2(uint16_t D) // 30纳秒ns  根据手册要用到IIC的HS高速模式
{
    while (--D)
        ;
}
void SCL_OUT_3221_2(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
 
    RCC_APB2PeriphClockCmd(INA219_I2C_GPIO_CLOCK, ENABLE);
    
    /* Configure I2C1 pins: PB12->SCL->OUT */
    GPIO_InitStructure.GPIO_Pin = INA3221_2_SCL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(INA3221_2_SCL_PORT, &GPIO_InitStructure);
    GPIO_SetBits(INA3221_2_SCL_PORT, INA3221_2_SCL_PIN);
	 
}
void SDA_OUT_3221_2(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
 
    RCC_APB2PeriphClockCmd(INA219_I2C_GPIO_CLOCK, ENABLE);
    
    /* Configure I2C1 pins: PB14->SDA-OUT */
    GPIO_InitStructure.GPIO_Pin = INA3221_2_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(INA3221_2_SDA_PORT, &GPIO_InitStructure);
    GPIO_SetBits(INA3221_2_SDA_PORT, INA3221_2_SDA_PIN);
	    

}
 
void SDA_IN_3221_2(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
 
    RCC_APB2PeriphClockCmd(INA219_I2C_GPIO_CLOCK, ENABLE);
    
    /* Configure I2C1 pins: PB14->SDA-IN */
    GPIO_InitStructure.GPIO_Pin = INA3221_2_SDA_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(INA3221_2_SDA_PORT, &GPIO_InitStructure);
	
}
/**
 * @description: stm32f103c8t6 微秒延迟函数 72Mhz
 * @param {uint16_t} us
 * @return {*}
 */
void delay_nus_3221_2(uint16_t us)
{
    uint32_t temp;
    SysTick->LOAD = us * (72000000 / 8000000); //时间加载
    SysTick->VAL = 0x00;                       //清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;  //开始倒数
    do
    {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16))); //等待时间到达
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;      //关闭计数器
    SysTick->VAL = 0X00;                            //清空计数器
}

/************************模拟IIC接口****************************/
void INA3221_2_IIC_Init(void)
{
    // IO初始化设置由cubeMX完成 移植的话需要在此处实现IO初始化
	  SCL_OUT_3221_2();
    SDA_OUT_3221_2();

    IIC_SDA_H_3221_2;
    IIC_SCL_H_3221_2;

    delay_nms_3221_2(5);
}
void INA3221_2_IIC_Start(void)
{
    IIC_SDA_H_3221_2;
    IIC_SCL_H_3221_2;
    delay_nns_3221_2(5);
    IIC_SDA_L_3221_2; // START:when CLK is high,DATA change form high to low
    delay_nns_3221_2(5);
    IIC_SCL_L_3221_2; //钳住I2C总线，准备发送或接收数据
    delay_nns_3221_2(5);
}

void INA3221_2_IIC_Stop(void)
{
    IIC_SDA_L_3221_2; // STOP:when CLK is high DATA change form low to high
    delay_nns_3221_2(5);
    IIC_SCL_H_3221_2;
    delay_nns_3221_2(5);
    IIC_SDA_H_3221_2; //发送I2C总线结束信号
    delay_nns_3221_2(5);
}

void INA3221_2_IIC_Ack(void)
{
    IIC_SDA_L_3221_2;
    delay_nns_3221_2(5);
    IIC_SCL_H_3221_2;
    delay_nns_3221_2(5);
    IIC_SCL_L_3221_2;
    delay_nns_3221_2(5);
    IIC_SDA_H_3221_2;
}

void INA3221_2_IIC_NAck(void)
{
    IIC_SDA_H_3221_2;
    delay_nns_3221_2(5);
    IIC_SCL_H_3221_2;
    delay_nns_3221_2(5);
    IIC_SCL_L_3221_2;
    delay_nns_3221_2(5);
    IIC_SDA_L_3221_2;
}

uint8_t INA3221_2_IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    IIC_SDA_H_3221_2;
    SDA_IN_3221_2(); // SDA设置为输入
    delay_nns_3221_2(5);
    IIC_SCL_H_3221_2;
    delay_nns_3221_2(5);

    while (SDA_READ_3221_2)
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            INA3221_2_IIC_Stop();
            return 1;
        }
    }
    IIC_SCL_L_3221_2; //时钟输出0
    SDA_OUT_3221_2(); // SDA设置为输入
    return 0;
}

void INA3221_2_IIC_Send_Byte(uint8_t txd)
{	
		uint8_t i = 0;
    IIC_SCL_L_3221_2; //拉低时钟开始数据传输
	
    for (i = 0; i < 8; i++)
    {
        if (txd & 0x80)
            IIC_SDA_H_3221_2;
        else
            IIC_SDA_L_3221_2;
        txd <<= 1;

        IIC_SCL_H_3221_2;
        delay_nns_3221_2(5);
        IIC_SCL_L_3221_2;
        delay_nns_3221_2(5);
    }
    //    IIC_SDA_H_3221_2;
    //    IIC_SCL_H_3221_2;            //当去掉wait_ack时的时候添加
    //    delay_nns_3221_2(5);
    // IIC_SCL_L_3221_2;
}

uint8_t INA3221_2_IIC_Read_Byte(unsigned char ack)
{
    uint8_t TData = 0, i;
    SDA_IN_3221_2(); // SDA设置为输入
    for (i = 0; i < 8; i++)
    {
        IIC_SCL_H_3221_2;
        delay_nns_3221_2(5);
        TData = TData << 1;
        // if(GPIOB->IDR& GPIO_IDR_IDR7)        //判断SDA是否为高
        if (SDA_READ_3221_2)
        {
            TData |= 0x01;
        }
        IIC_SCL_L_3221_2;
        delay_nns_3221_2(5);
    }
    SDA_OUT_3221_2(); // SDA设置为输出
    if (!ack)
        INA3221_2_IIC_NAck();
    else
        INA3221_2_IIC_Ack();

    return TData;
}


void INA3221_2_SendData(uint8_t addr, uint8_t reg, uint16_t data)
{
    uint8_t temp = 0;
    INA3221_2_IIC_Start();
    INA3221_2_IIC_Send_Byte(addr);
    INA3221_2_IIC_Wait_Ack();

    INA3221_2_IIC_Send_Byte(reg);
    INA3221_2_IIC_Wait_Ack();

    temp = (uint8_t)(data >> 8);
    INA3221_2_IIC_Send_Byte(temp);
    INA3221_2_IIC_Wait_Ack();

    temp = (uint8_t)(data & 0x00FF);
    INA3221_2_IIC_Send_Byte(temp);
    INA3221_2_IIC_Wait_Ack();

    INA3221_2_IIC_Stop();
}

void INA3221_2_SetRegPointer(uint8_t addr, uint8_t reg)
{
    INA3221_2_IIC_Start();

    INA3221_2_IIC_Send_Byte(addr);
    INA3221_2_IIC_Wait_Ack();

    INA3221_2_IIC_Send_Byte(reg);
    INA3221_2_IIC_Wait_Ack();

    INA3221_2_IIC_Stop();
}

uint16_t INA3221_2_ReadData(uint8_t addr)
{
    uint16_t temp = 0;
    INA3221_2_IIC_Start();

    INA3221_2_IIC_Send_Byte(addr + 1);
    INA3221_2_IIC_Wait_Ack();

    temp = INA3221_2_IIC_Read_Byte(1);
    temp <<= 8;
    temp |= INA3221_2_IIC_Read_Byte(0);

    INA3221_2_IIC_Stop();
    return temp;
}


/**
 * @description: INA3221_2自检
 * @param {uint8_t} addr
 * @return {*}
 */
void INA3221_2_SelfCheck(uint8_t addr)
{
    uint16_t id = 0;
    while (id != INA3221_2_DIE_ID)
    {
        delay_nms_3221_2(50);
        //卡这说明硬件连接异常或者是地址错误
        INA3221_2_SetRegPointer(addr, INA3221_2_DIEID_REG);
        id = INA3221_2_ReadData(addr);
    }
}

/********************************应用部分****************************************/

/**
 * @description: INA3221_2初始化
 * @return {*}
 */
void INA3221_2_Init(uint8_t addr)
{
    
    INA3221_2_IIC_Init();     //初始化IIC

    INA3221_2_SendData(addr, INA3221_2_CFG_REG, 0x8000); //软件复位
    delay_ms(10);

    // 配置寄存器设置控制三个输入通道的分流和总线电压测量的工作模式。 
    // 该寄存器控制分流和总线电压测量的转换时间设置以及使用的平均模式。 
    // 配置寄存器用于独立启用或禁用每个通道，以及选择控制选择要测量的信号的操作模式。
    // 详见数据手册P26
    INA3221_2_SendData(addr, INA3221_2_CFG_REG, 0x7127); // 7127为默认配置
	   //INA3221_2_SendData(addr, INA3221_2_CFG_REG, 0x74E7); //采样时间8ms,运作良好，但响应速度慢
	  //INA3221_2_SendData(addr, INA3221_2_CFG_REG, 0x72E7);//最终选择
    INA3221_2_SelfCheck(addr);
     
}

/**
 * @description: 获取电压
 * @param {uint8_t} addr
 * @param {uint8_t} channel 通道编号(1\2\3)
 * @return {uint16_t} 通道所对应的电压
 */
uint16_t INA3221_2_GetVoltage(uint8_t addr, uint8_t channel)
{
    uint32_t temp = 0;
    switch (channel)
    {
    case 1:
        INA3221_2_SetRegPointer(addr, INA3221_2_CH1BUS_REG);
        break;
    case 2:
        INA3221_2_SetRegPointer(addr, INA3221_2_CH2BUS_REG);
        break;
    case 3:
        INA3221_2_SetRegPointer(addr, INA3221_2_CH3BUS_REG);
        break;
    default:
        break;
    }
    temp = INA3221_2_ReadData(addr);
    if (temp & 0x8000)
        temp = ~(temp - 1);
    return (uint16_t)temp;
}


/**
 * @description: 获取分流电压
 * @param {uint8_t} addr    ina3221_2的IIC地址
 * @param {uint8_t} channel 通道编号(1\2\3)
 * @return {uint16_t} 通道的分流电压
 */
int INA3221_2_GetShuntVoltage(uint8_t addr, uint8_t channel)
{
    uint32_t temp = 0;
    switch (channel)  
    {
    case 1:
        INA3221_2_SetRegPointer(addr, INA3221_2_CH1SHUNT_REG);
        break;
    case 2:
        INA3221_2_SetRegPointer(addr, INA3221_2_CH2SHUNT_REG);
        break;
    case 3:
        INA3221_2_SetRegPointer(addr, INA3221_2_CH3SHUNT_REG);
        break;
    default:
        break;
    }
    temp = INA3221_2_ReadData(addr);

		//delay_ms(1);//新添加，看看之后会不会出错
		
    if (temp & 0x8000){  //如果为负
        temp = ~(temp - 1);
				//temp *= -1;
			return (uint16_t)temp*(-1);
		}
    return (uint16_t)temp;
}

int INA3221_2_Mean_filtering1(void){
	const int count=5; //均值滤波的数据量
	int i=0;
	int sum=0;
	for(i=0;i<count;i++)
		sum+=INA3221_2_GetShuntVoltage(0x80,1);
	return sum/count;
}
 //递增 的快排算法
int compInc_2(const void *a, const void *b)  
{  
    return *(int *)a - *(int *)b;  
} 
//对获取的数据进行中值滤波
int INA3221_2_Median_filtering1(void)
{	int i;
	s32 a;
	const int count=5; //中值滤波的数据量
	/*
  static s32 nums[count]; //可能需要对数组初始化,debug看一下初始值是否为0;
	s32 nums1[count];
	for(i=0;i<count-1;i++)
		nums[i]=nums[i+1];
	nums[count-1]=INA3221_2_GetShuntVoltage(0x80,1);
	
	for(i=0;i<count;i++)
		nums1[i]=nums[i];
	 qsort(nums1, count, sizeof(nums1[0]), compInc_2); //排序
	if(count%2==0) {          //奇偶取中位数的方法不一样
            return (nums1[(count/2)-1]+nums1[(count/2)]);
         }
        else {
            return nums1[count/2];
        } 
	*/
	s32 nums[count]; //可能需要对数组初始化,debug看一下初始值是否为0;
	s32 nums1[count];
	for(i=0;i<count;i++)
		nums[i]=INA3221_2_GetShuntVoltage(0x80,1);
	for(i=0;i<count;i++)
		nums1[i]=nums[i];
	 qsort(nums1, count, sizeof(nums1[0]), compInc_2); //排序
	if(count%2==0) {          //奇偶取中位数的方法不一样
            return (nums1[(count/2)-1]+nums1[(count/2)]);
         }
        else {
            return nums1[count/2];
        } 
	
}
int INA3221_2_Median_filtering2(void)
{	int i;
	s32 a;
	const int count=5; //中值滤波的数据量
	/*
  static s32 nums[count]; //可能需要对数组初始化,debug看一下初始值是否为0;
	s32 nums1[count];
	for(i=0;i<count-1;i++)
		nums[i]=nums[i+1];
	nums[count-1]=INA3221_2_GetShuntVoltage(0x80,2);
	
	for(i=0;i<count;i++)
		nums1[i]=nums[i];
	 qsort(nums1, count, sizeof(nums1[0]), compInc_2); //排序
	if(count%2==0) {          //奇偶取中位数的方法不一样
            return (nums1[(count/2)-1]+nums1[(count/2)]);
         }
        else {
            return nums1[count/2];
        } 
	*/
	s32 nums[count]; //可能需要对数组初始化,debug看一下初始值是否为0;
	s32 nums1[count];
	for(i=0;i<count;i++)
		nums[i]=INA3221_2_GetShuntVoltage(0x80,2);
	for(i=0;i<count;i++)
		nums1[i]=nums[i];
	 qsort(nums1, count, sizeof(nums1[0]), compInc_2); //排序
	if(count%2==0) {          //奇偶取中位数的方法不一样
            return (nums1[(count/2)-1]+nums1[(count/2)]);
         }
        else {
            return nums1[count/2];
        } 
	
}
int INA3221_2_Median_filtering3(void)
{	int i;
	s32 a;
	const int count=5; //中值滤波的数据量
	/*
  static s32 nums[count]; //可能需要对数组初始化,debug看一下初始值是否为0;
	s32 nums1[count];
	for(i=0;i<count-1;i++)
		nums[i]=nums[i+1];
	nums[count-1]=INA3221_2_GetShuntVoltage(0x80,3);
	
	for(i=0;i<count;i++)
		nums1[i]=nums[i];
	 qsort(nums1, count, sizeof(nums1[0]), compInc_2); //排序
	if(count%2==0) {          //奇偶取中位数的方法不一样
            return (nums1[(count/2)-1]+nums1[(count/2)]);
         }
        else {
            return nums1[count/2];
        } 
	*/
	s32 nums[count]; //可能需要对数组初始化,debug看一下初始值是否为0;
	s32 nums1[count];
	for(i=0;i<count;i++)
		nums[i]=INA3221_2_GetShuntVoltage(0x80,3);
	for(i=0;i<count;i++)
		nums1[i]=nums[i];
	 qsort(nums1, count, sizeof(nums1[0]), compInc_2); //排序
	if(count%2==0) {          //奇偶取中位数的方法不一样
            return (nums1[(count/2)-1]+nums1[(count/2)]);
         }
        else {
            return nums1[count/2];
        } 
	
}


/* gpio引脚初始化 -------------------------------------*/
// 定义GPIO引脚
#define GPIOF_PINS (GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10)
#define GPIOA_PINS (GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_6 | GPIO_Pin_7)
#define GPIOB_PINS (GPIO_Pin_12 | GPIO_Pin_11)

// 定义GPIO端口
#define GPIO_PORTF GPIOF
#define GPIO_PORTA GPIOA
#define GPIO_PORTB GPIOB

// 定义GPIO模式速度
#define GPIO_MODE_OUTPUT GPIO_Mode_Out_PP
#define GPIO_SPEED GPIO_Speed_50MHz


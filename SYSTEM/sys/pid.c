//pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
#include "pid.h"
#include <stdio.h>
 
//static PID pid;
 
void PID_Init(PID pid)
{
    pid.kp = 0.1;
    pid.ki = 0.2;
    pid.kd = 0.3;
    pid.limit = 300;
	  pid.K1=0;
	  pid.K2=0.1;
    pid.ek = 0;
    pid.ek_1 = 0;
	  pid.ek_2 = 0;  
    pid.ek_sum = 0;
}
 
// 位置式PID控制
float PID_Postion(PID pid,int Encoder,int Target)
{
    float pwm = 0;
    pid.ek = Target - Encoder; // 计算当前误差
    pid.ek_sum += pid.ek;      //求出偏差的积分
    pwm = pid.kp*pid.ek + pid.ki*pid.ek_sum + pid.kd*(pid.ek - pid.ek_1);   //位置式PID控制器
    pid.ek_1 = pid.ek;   //保存上一次偏差 
    if(pwm > pid.limit)
    {
      pwm =  pid.limit;
    }
    else if(pwm < -pid.limit)
    {
      pwm =  -pid.limit;
    }
    return pwm;
}


// 增量式PID控制
float PID_Increase(PID pid,int Encoder,int Target)
{
    float pwm = 0;
    pid.ek = Target - Encoder; // 计算当前误差
    pid.ek_sum += pid.ek;      //求出偏差的积分
		if(pid.ek<pid.K2&&){     //
		 pid.ek_sum=0;
		}
    pwm = pid.kp*(pid.ek - pid.ek_1) + pid.ki*pid.ek + pid.kd*(pid.ek - 2*pid.ek_1 + pid.ek_2);   //增量式PID控制器
    pid.ek_2 = pid.ek_1; //保存上上一次的偏差
	  pid.ek_1 = pid.ek;   //保存上一次偏差   
    if(pwm > pid.limit)
    {
      pwm =  pid.limit;
    }
    else if(pwm < -pid.limit)
    {
      pwm =  -pid.limit;
    }
    return pwm;
	}

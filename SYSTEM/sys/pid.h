#ifndef __PID_H
#define __PID_H 	
#include <stdio.h>
typedef struct PID
{ 
  float kp;
  float ki;
  float kd;
  float ek;     //当前误差
  float ek_1;   //上一次误差
  float ek_2;  //上上一次误差
  float ek_sum; //误差总和
  float limit;  //限幅
	double K1;//阈值1，闭环死区是指执行机构的最小控制量，无法再通过调节来满足控制精度，如果仍然持续调节，系统则会在目标值前后频繁动作，不能稳定下来
	double K2;//阈值2，通过积分分离的方式来实现抗积分饱和，积分饱和是指执行机构达到极限输出能力了，仍无法到达目标值，在很长一段时间内无法消除静差造成的。
}PID;


void PID_Init(PID pid);
float PID_Postion(PID pid,int Encoder,int Target);
float PID_Increase(PID pid,int Encoder,int Target);
#endif
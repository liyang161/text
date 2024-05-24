#include "NeuralPID.h"
#include "math.h"
/*定义结构体和公用体*/


/*单神经元学习规则函数*/
static void NeureLearningRules(NEURALPID *vPID,double zk,double uk,double *xi)
{	

  vPID->wi=vPID->wi+vPID->ki*zk*uk*xi[0];
  vPID->wp=vPID->wp+vPID->kp*zk*uk*xi[1];
  vPID->wd=vPID->wd+vPID->kd*zk*uk*xi[2];
	if( vPID->wi==0)  vPID->wi=0.1;
	if( vPID->wp==0)  vPID->wp=0.1;
	if( vPID->wd==0)  vPID->wd=0.1;
	
	if( vPID->wi>400)  vPID->wi=400;
	
	
}

/* 单神经元PID初始化操作,需在对vPID对象的值进行修改前完成                     */
/* NEURALPID vPID，单神经元PID对象变量，实现数据交换与保存                    */
/* float vMax,float vMin，过程变量的最大最小值（量程范围）                    */
void NeuralPIDInitialization(NEURALPID *vPID,double vMax,double vMin)
{
  vPID->setpoint=vMin;                  /*设定值*/
  
  vPID->kcoef=1; /*神经元输出比例*/
  vPID->kp=0.05;                         /*比例学习速度*/
  vPID->ki=0.05;                        /*积分学习速度*/
  vPID->kd=0.05;                         /*微分学习速度*/
  
  vPID->lasterror=0.0;                  /*前一拍偏差*/
  vPID->preerror=0.0;                   /*前两拍偏差*/
  vPID->result=vMin;                    /*PID控制器结果*/
  vPID->output=0.0;                     /*输出值，百分比*/
 
  vPID->maximum=vMax;                   /*输出值上限*/
  vPID->minimum=vMin;                   /*输出值下限*/  
  vPID->deadband=0.03;    /*死区*/
 
  vPID->wp=0.10; /*比例加权系数*/
  vPID->wi=0.10; /*积分加权系数*/
  vPID->wd=0.10; /*微分加权系数*/
}

/* 神经网络参数自整定PID控制器，以增量型方式实现                              */
/* NEURALPID vPID，神经网络PID对象变量，实现数据交换与保存                    */
/* float pv，过程测量值，对象响应的测量数据，用于控制反馈                     */
void NeuralPID(NEURALPID *vPID,double pv)
{
  double x[3];
  double w[3];
  double sabs;
  double error;
  double result;
  double deltaResult;
 
  error=vPID->setpoint-pv;
  result=vPID->result;
	

  if(fabs(error)>vPID->deadband)
  {
    x[0]=error;
    x[1]=error-vPID->lasterror;
    x[2]=error-vPID->lasterror*2+vPID->preerror;
  
    sabs=fabs(vPID->wi)+fabs(vPID->wp)+fabs(vPID->wd);
    //w[0]=vPID->wi/sabs;
    //w[1]=vPID->wp/sabs;
   // w[2]=vPID->wd/sabs;
		w[0]=vPID->wi;
    w[1]=vPID->wp;
    w[2]=vPID->wd;
		
    
    deltaResult=(w[0]*x[0]+w[1]*x[1]+w[2]*x[2])*vPID->kcoef;
  }
  else
  {
    deltaResult=0;
		return ;
  }
 
  result=result+deltaResult;
  if(result>vPID->maximum)
  {
    result=vPID->maximum;
  }
  if(result<vPID->minimum)
  {
    result=vPID->minimum;
  }
  vPID->result=result;
  vPID->output=(vPID->result-vPID->minimum)*100/(vPID->maximum-vPID->minimum);
 
  //单神经元学习
  NeureLearningRules(vPID,error,result,x);
  
  vPID->preerror=vPID->lasterror;
  vPID->lasterror=error;
}


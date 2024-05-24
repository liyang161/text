#ifndef NEURAL_H
#define NEURAL_H

typedef struct
{
  double setpoint;               /*设定值*/
  double kcoef;                  /*神经元输出比例*/
  double kp;                     /*比例学习速度*/
  double ki;                     /*积分学习速度*/
  double kd;                     /*微分学习速度*/
  double lasterror;              /*前一拍偏差*/
  double preerror;               /*前两拍偏差*/
  double deadband;               /*死区*/
  double result;                 /*输出值*/
  double output;                 /*百分比输出值*/
  double maximum;                /*输出值的上限*/
  double minimum;                /*输出值的下限*/
  double wp;                     /*比例加权系数*/
  double wi;                     /*积分加权系数*/
  double wd;                     /*微分加权系数*/
}NEURALPID;
static void NeureLearningRules(NEURALPID *vPID,double zk,double uk,double *xi);
void NeuralPID(NEURALPID *vPID,double pv);
void NeuralPIDInitialization(NEURALPID *vPID,double vMax,double vMin);


#endif
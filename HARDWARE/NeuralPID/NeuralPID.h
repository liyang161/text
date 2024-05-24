#ifndef NEURAL_H
#define NEURAL_H

typedef struct
{
  double setpoint;               /*�趨ֵ*/
  double kcoef;                  /*��Ԫ�������*/
  double kp;                     /*����ѧϰ�ٶ�*/
  double ki;                     /*����ѧϰ�ٶ�*/
  double kd;                     /*΢��ѧϰ�ٶ�*/
  double lasterror;              /*ǰһ��ƫ��*/
  double preerror;               /*ǰ����ƫ��*/
  double deadband;               /*����*/
  double result;                 /*���ֵ*/
  double output;                 /*�ٷֱ����ֵ*/
  double maximum;                /*���ֵ������*/
  double minimum;                /*���ֵ������*/
  double wp;                     /*������Ȩϵ��*/
  double wi;                     /*���ּ�Ȩϵ��*/
  double wd;                     /*΢�ּ�Ȩϵ��*/
}NEURALPID;
static void NeureLearningRules(NEURALPID *vPID,double zk,double uk,double *xi);
void NeuralPID(NEURALPID *vPID,double pv);
void NeuralPIDInitialization(NEURALPID *vPID,double vMax,double vMin);


#endif
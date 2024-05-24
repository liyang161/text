#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "park.h"
#include <adc.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "stack.h"
#include "filter.h"
#include "parameter_identify.h"
#include "FluxObserver.h"
#include "SMO.h"
#include "INA3221.h"
#include "INA3221_2.h"
#include "IQmathLib.h"
#include "configure.h"
#define PI 3.14159

#define get_current1 INA3221_2_Median_filtering2()*40.0/1000/1000*5
#define get_current2 INA3221_Median_filtering2()*40.0/1000/1000*5
#define get_current3 -1*INA3221_Median_filtering1()*40.0/1000/1000*5
#define get_current4 -1*INA3221_2_Median_filtering1()*40.0/1000/1000*5

			
#define SEND_BUF_SIZE 100	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.

u8 SendBuff[SEND_BUF_SIZE];	//�������ݻ�����
const u8 TEXT_TO_SEND[]={"1.111,1.111,1.111,1.111,1.111,1.111\n"};
int get_length(char str[]);
int voltage_source=10;//��Դ��ѹ
int PWM_T=4000;//PWM����
double phase_dif=3*PI/4;//��λ��
//double angel=2.2+3.141;
double angel=5.25-3.14;
int method=0;//0Ϊ����ֱ���˶���������ʱ��仯��1Ϊ�����㶨ģʽ
int delaytime=5;
int voltage_amp=0;

double phase=0;    //��ǰ����λ��  x100
double last_phase=0;
int get_hall=-1;      //�Ƿ�����λ�����ݻ�����������Ϣ�ı�־λ
int get_cur=-1;      //�Ƿ�����λ�����ݵ�����������Ϣ�ı�־λ
double hall1;      //��������������
double hall2;
double current_1=0;
double current_2=0;
double current_3=0;
double current_4=0;
double ialphi=0;
double ibeta=0;
double id=0;
double iq=0;
double ualphi=0;
double ubeta=0;
double ud=0;
double uq=0;
double runTime=0;//whileѭ������һ�ε�ʱ�䣬��λ΢��

int start=-1;     //��ʼ��־λ
double position=0;  //��ǰλ��
static double last_position=0;
double speed=0;    //��ǰ�ٶ�
double last_speed=0;  
double speed_filter=0;    //�˲����ٶ�
double last_speed_filter=0; 
double acceleration=0;//��ǰ���ٶ�
double acceleration_filter=0;//��ǰ���ٶ�
double force_e=0;//�����
double force_contact=0;//�Ӵ���

double T=50;    //��ǰ�����������Ϊ 50mm
double current=0;//��ǰ����
double current226=0;//��ǰ����

//double kp_current=100,ki_current=1,kd_current=20; //�������Ƶ�pidϵ��
double kp_current=0,ki_current=0,kd_current=0; //�������Ƶ�pidϵ�� 
//double kp_current=0,ki_current=10,kd_current=0; 

double PID_value=0;
double PID_value_Q=0;
double PID_value_D=0;
double PID_value_force=0;
double light_ruler_position=0;
double light_ruler_phase=0;
int light_ruler_count=0;

double R_add=1;
double L_add=1;
double Ke_add=1;

time_t c_start;   //��¼��������ʱ��
//I = 0.1564V^2 + 0.0122V + 0.0455   //���豸��ѹ�������ϵ
/*
ͨ��ָ�
for  1     �������ѹ��Ϊ1N
vos  24    ����voltage_sourceΪ24V
vol 500    ���ĵ�ǰ��ѹ�����ֵΪ5.00V
ang 314    ���ĳ�ʼ��λ��Ϊ3.14��rad��
dif 314		 ������λ��Ϊ3.14��rad��
pha 314    ���ĵ�ǰ��λΪ3.14��rad��
del 10     ������ʱΪ10����
ckp 1       ����kpϵ��Ϊ1
cki 1			 ����kiϵ��Ϊ1
ckd 1       ����kdϵ��Ϊ1
gethal        ��ȡ������������Ϣ��λ����Ϣ
getcur     ��ȡ��ǰ������Ϣ
*/


double tri_wave(){
	static double result=0;
	static int a=1;
	double add=0.001;//ÿ�ε�����
	
	result=result+add*a;
	if(result>voltage_amp/100.0){
		a=a*-1;
		delay_ms(1000);
	}
	if(result<=0){
		a=a*-1;
		result=0;
	}
	return result;

}
double pidController(double error) {
		double output=0;
		double proportional;
		double derivative;
    // PID����
    double kp = 1000;  // ����ϵ��
    double ki = 5;  // ����ϵ��
    double kd = 0;  // ΢��ϵ��
    
    // PID����
    static double integral = 0;  // ������
    static double previous_error = 0;  // ��һ�����
    
	//if(error<0.05&&error>-0.05&&abs(speed_filter)>3){ //ֻ������Сʱ���ã���ֹӰ���Ծ��Ӧ
	//if(error<0.05&&error>-0.05){ //ֻ������Сʱ���ã���ֹӰ���Ծ��Ӧ
    // ����PID������
    proportional = kp * error;  // ����������
    integral += error;  // ���ֿ�����
    derivative = kd * (error - previous_error);  // ΢�ֿ�����
    
    // ������һ�����
    previous_error = error;
    
    // ����PID���
     output = proportional + (ki * integral) + derivative;
		
		 
	//}
	//printf("%lf,%lf,%lf,%lf\n",error,iq,output,speed_filter);
    
    return output;
}

//��ֵ�˲���
double medianFilter3_1(double newSample) {
    // ��̬�������ڱ��������3��ֵ
    static double samples[3] = {0.0, 0.0, 0.0};
    // ��̬�������ڼ�¼������Ӧ�ò����λ��
    static int index = 0;
    double median;

    // �����������뵽��̬����
    samples[index] = newSample;
    // ����������ѭ��ʹ��3��λ��
    index = (index + 1) % 3;

    // ȷ����ֵ
    if ((samples[0] <= samples[1] && samples[0] >= samples[2]) || (samples[0] >= samples[1] && samples[0] <= samples[2])) {
        median = samples[0];
    } else if ((samples[1] <= samples[0] && samples[1] >= samples[2]) || (samples[1] >= samples[0] && samples[1] <= samples[2])) {
        median = samples[1];
    } else {
        median = samples[2];
    }

    return median;
}
//��ֵ�˲���
double medianFilter3_2(double newSample) {
    // ��̬�������ڱ��������3��ֵ
    static double samples[3] = {0.0, 0.0, 0.0};
    // ��̬�������ڼ�¼������Ӧ�ò����λ��
    static int index = 0;
    double median;

    // �����������뵽��̬����
    samples[index] = newSample;
    // ����������ѭ��ʹ��3��λ��
    index = (index + 1) % 3;

    // ȷ����ֵ
    if ((samples[0] <= samples[1] && samples[0] >= samples[2]) || (samples[0] >= samples[1] && samples[0] <= samples[2])) {
        median = samples[0];
    } else if ((samples[1] <= samples[0] && samples[1] >= samples[2]) || (samples[1] >= samples[0] && samples[1] <= samples[2])) {
        median = samples[1];
    } else {
        median = samples[2];
    }

    return median;
}
//ͨ��DMA���д���
void print(char* charTemp , ...) 
{
    u32 num = 0;
	int i=0;
    char* t=charTemp ;    
    while(*t++ !=0)  num++;
	for(i=0;i<num;i++)
	SendBuff[i]=charTemp[i];
 USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //ʹ�ܴ���1��DMA����      
	MYDMA_Enable(DMA1_Channel4,num);//��ʼһ��DMA���䣡	 
	
}

void  force_observer(){ //�Ӵ����۲���
	//1A  1.3N
	double m=0.6;//��������
	double K1=1.3;//�����������ı���ϵ��
	//force_contact-force_e=m*acceleration;
	force_e=iq*K1;
	if(abs(acceleration_filter)>100)
	force_contact=m*acceleration_filter/1000+iq*K1;
	else force_contact=iq*K1;
	//force_contact=m*acceleration/1000;  //acceleration�ĵ�λ�Ǻ���ÿ�룬��Ҫת��Ϊ��ÿ��
	//printf("%lf,%lf,%lf\n",acceleration_filter,force_e,force_contact);
	//printf("%lf,%lf,%lf\n",acceleration_filter,speed_filter,position);		
}
int prediction(double w,double id,double iq,double id_ref,double iq_ref,double ang){ //���SVPWM�ķ���
	double R=6.76*0.1;//����     //��Ҫ����Ӧ�������޸Ĳ���
	double L=-3.856*0.001;//���
	double Ke=1.27*0.01;//��ͨ
	double Ts=0.02;
	//sw Ϊ�������״̬�ռ�
	static int Sw[16][4] = {{0,0,0,0},{0,0,0,1},{0,0,1,0},{0,0,1,1},{0,1,0,0},{0,1,0,1},{0,1,1,0},{0,1,1,1},{1,0,0,0},{1,0,0,1},{1,0,1,0},{1,0,1,1},{1,1,0,0},{1,1,0,1},{1,1,1,0},{1,1,1,1}};
	int i;	
	double Vdc=10;   //��Դ��ѹΪ10V
	double Vd,Vq,Va,Vb,did,diq,id_next,iq_next;
	double err[16];
	int min,min_value; 

	
	for(i=0;i<16;i++){  //����һ�飬�������ֶ���ʹ������С
		Four_phase_clark(Sw[i][0],Sw[i][1],Sw[i][2],Sw[i][3],&Va,&Vb);
		Park(Va,Vb,ang,&Vd,&Vq);
		
		did = 1/L*(Vd-R*id+w*L*iq)*Ts;
    diq = 1/L*(Vq-R*iq-w*(Ke+L*id))*Ts;
		
		id_next = id + did;
    iq_next = iq + diq;
		
		err[i] = (id_next-id_ref)*(id_next-id_ref)+(iq_next-iq_ref)*(iq_next-iq_ref);
	}
	
	//Ѱ��err�е���Сֵ
	min=0;
	min_value=err[0];
	for(i=1;i<16;i++){
		if(err[i]<min_value)
			min=i;
	}
	return i;
	
}

//D�����PID����
int D_current_PID(double target_current,double now_current,double PWM_value) { // Ŀ���������ǰ��������ǰPWMֵ��������������D���ѹ//95,40,x    125,40,x
    double max=300,min=-300;//���������Сֵ�����ⷢ��Σ��
	  //static double kp=0,ki=10,kd=0;
	  double kp=kp_current,ki=ki_current,kd=kd_current;
    static int count=0;
    double K1=0.03;//��ֵ���ջ�������ִָ�л�������С���������޷���ͨ��������������ƾ��ȣ������Ȼ�������ڣ�ϵͳ�����Ŀ��ֵǰ��Ƶ�������������ȶ�����
    double K2=0.2;//ͨ�����ַ���ķ�ʽ��ʵ�ֿ����ֱ��ͣ����ֱ�����ִָ�л����ﵽ������������ˣ����޷�����Ŀ��ֵ���ںܳ�һ��ʱ�����޷�����������ɵġ�
    //������ֱ��͵�һ�ַ�����ʹ�û��ַ��룬�÷��������ۼ����С��ĳ����ֵ��ʹ�û�����ۼ����������ټ����ۼ����൱��ֻʹ����PD��������
    static double current_lasterror=0;
	  static double current_lastlasterror=0;
    double current_error;//�洢Ŀ������͵�ǰ�����Ĳ�ֵ
    static double current_error_accumulation=0;//�洢Ŀ������͵�ǰ�����Ĳ�ֵ֮�ͣ�������
		double ia,ib,id,iq; 	
	
		
	  current_lastlasterror=current_lasterror;
		current_lasterror=target_current-now_current;
	  
	
		Four_phase_clark(current_1,current_2,current_3,current_4,&ia,&ib);
		Park(ialphi,ibeta,phase/100.0+angel,&id,&iq);
	  now_current=id;
	  //current_1=now_current; //���µ���ֵ������ֵֻ��Ҫ��PID�׶θ���

		

    //PID_value=0;
    current_error=target_current-now_current;

    if(current_error>K1||current_error<K1*(-1)) {    
				PWM_value+=(kp*(current_error-current_lasterror)+ki*current_error+kd*(current_error-2*current_lasterror+current_lastlasterror));
				if(PWM_value>max)
            PWM_value=max;
        if(PWM_value<min)
            PWM_value=min;

    }

			return PWM_value;
}

//Q�����PID����
int Q_current_PID(double target_current,double now_current,double PWM_value) { // Ŀ���������ǰ��������ǰPWMֵ��������������Q���ѹ//95,40,x    125,40,x
    double max=600,min=-600;//���������Сֵ�����ⷢ��Σ��
	  //static double kp=0,ki=10,kd=0;
	  double kp=0,ki=1500,kd=0;
    static int count=0;
    double K1=0.03;//��ֵ���ջ�������ִָ�л�������С���������޷���ͨ��������������ƾ��ȣ������Ȼ�������ڣ�ϵͳ�����Ŀ��ֵǰ��Ƶ�������������ȶ�����
    double K2=0.2;//ͨ�����ַ���ķ�ʽ��ʵ�ֿ����ֱ��ͣ����ֱ�����ִָ�л����ﵽ������������ˣ����޷�����Ŀ��ֵ���ںܳ�һ��ʱ�����޷�����������ɵġ�
    //������ֱ��͵�һ�ַ�����ʹ�û��ַ��룬�÷��������ۼ����С��ĳ����ֵ��ʹ�û�����ۼ����������ټ����ۼ����൱��ֻʹ����PD��������
    static double current_lasterror=0;
	  static double current_lastlasterror=0;
    double current_error;//�洢Ŀ������͵�ǰ�����Ĳ�ֵ
    static double current_error_accumulation=0;//�洢Ŀ������͵�ǰ�����Ĳ�ֵ֮�ͣ�������
		double ia,ib,id,iq; 	
	
	
	  current_lastlasterror=current_lasterror;
		current_lasterror=target_current-now_current;
	  
		
	  now_current=iq;
	  //current_1=now_current; //���µ���ֵ������ֵֻ��Ҫ��PID�׶θ���

		kp=kp_current;
		ki=ki_current;
		kd=kd_current;

    //PID_value=0;
    current_error=target_current-now_current;

    if(current_error>K1||current_error<K1*(-1)) {    
				PWM_value+=(kp*(current_error-current_lasterror)+ki*current_error+kd*(current_error-2*current_lasterror+current_lastlasterror));
				if(PWM_value>max)
            PWM_value=max;
        if(PWM_value<min)
            PWM_value=min;
    }
	//printf("%.3lf,%.3lf,%.3lf\n",target_current,now_current,PWM_value); 
			return PWM_value;
}

//����PID����
double force_PID(double a,double v) { // ���ٶȣ��ٶȣ���ǰPWMֵ������������//95,40,x    125,40,x
    double max=600,min=-600;//���������Сֵ�����ⷢ��Σ��
	  //static double kp=0,ki=10,kd=0;
	  double m=kp_current,d=ki_current;
    double K1=0.03;//��ֵ���ջ�������ִָ�л�������С���������޷���ͨ��������������ƾ��ȣ������Ȼ�������ڣ�ϵͳ�����Ŀ��ֵǰ��Ƶ�������������ȶ�����
    double K2=0.2;//ͨ�����ַ���ķ�ʽ��ʵ�ֿ����ֱ��ͣ����ֱ�����ִָ�л����ﵽ������������ˣ����޷�����Ŀ��ֵ���ںܳ�һ��ʱ�����޷�����������ɵġ�
		double PWM_value=0;	
	  
	if(abs(a)<300)
		a=0;
	if(abs(v)<10)
		v=0;
	
	PWM_value=a/1000.0*m+v/1000.0*d;

			return PWM_value;
}

//ģ��Ԥ�����
//id,iqΪ��������,idn,iqnΪ��ǰ����,w���ٶ�,ud,uqΪ���������
void MRAC(double id_aim,double iq_aim,double id_now,double iq_now,double w,double* ud_control,double* uq_control){
  double Ts=runTime/1000000;//����ʱ��
	//double R=2.98;//����     //��Ҫ����Ӧ�������޸Ĳ���
	//double L=0.034;//���
	//double Ke=0.06;//��ͨ
	static double index[]={0.0833, -0.0696,0.5917, 0.0149};
	int i=0;

	//static double R=0.76;
	static double R=1/0.2711;
	static double L=0.0135+0.004*0;
	static double Ke=1.27*0.01+0.005;//��ͨ
	Ts=0.02;//�ǵ�ɾ��  //����ʱ���ѡ�����Ҫ�����ɹ̶���֮��Ч���úܶ�
	
//	R=R*R_add;
//	L=L*L_add;
//	Ke=Ke*Ke_add;
//	R_add=1;
//	L_add=1;
//	Ke_add=1;
	//printf("R:%lf,L:%lf,Ke:%lf\n",R,L,Ke);

//	*uq=R*iqn+w*L*idn+L*(iq-iqn)/Ts+w*Ke;
//	*ud=-w*L*iqn+R*idn+L*(id-idn)/Ts;
	
	if(abs(w)<1)
		w=0;
	Ke=0.062;
	//Ke=0;
	*uq_control=R*iq_aim;
	*ud_control=R*id_aim;
	
//	*uq=R*R_add*iqn+w*L*L_add*idn+L*L_add*(iq-iqn)/Ts+w*Ke*Ke_add
//	;
//	*ud=-w*L*L_add*iqn+R*R_add*idn+L*L_add*(id-idn)/Ts;

	//printf("%.3lf,%.3lf,%.3lf\n",iq_aim,iq_now,*uq_control);
	if(*uq_control<=0)
		*uq_control=0;
	
	*uq_control=*uq_control*400;
	*ud_control=*ud_control*400;
	

}
/**
  * @brief  Start measure time.
  * @param  None.
  * @return None.
	https://blog.csdn.net/u012325601/article/details/78515538
  */
 void MeasureTimeStart(void)
{
  SysTick->CTRL |= SysTick_CLKSource_HCLK;  /* Set the SysTick clock source. */
  SysTick->LOAD  = 0xFFFFFF;                /* Time load (SysTick-> LOAD is 24bit). */
  SysTick->VAL   = 0xFFFFFF;                /* Empty the counter value. */
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; /* Start the countdown. */
  __nop();                                  /* Waiting for a machine cycle. */
}
 
/**
  * @brief  Stop measure time.
  * @param  [in] clock: System clock frequency(unit: MHz).
  * @return Program run time(unit: us).
  */
 double MeasureTimeStop(uint32_t clock)
{
	double time = 0.0;
  uint32_t count = SysTick->VAL;             /* Read the counter value. */
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; /* Close counter. */
  
  
  
  if(clock > 0)
  {
    time = (double)(0xFFFFFF - count) / (double)clock; /* Calculate program run time. */
  }
  
  return time;
}
double get_mid(double *nums, int size){
	double a[5];
	int i,j;
	double t;
  double mid;
	for(i=0;i<5;i++)
		a[i]=nums[i];
for (i=0;i<size-1;i++)//iΪ���������
{
	for(j=0;j<size-i-1;j++)//jΪ��i����ȽϵĴ���
	{
		if(a[j]>a[j+1])
		{
			t=a[j];
			a[j]=a[j+1];
			a[j+1]=t;
		}
	}
}

	mid=a[2];
return mid;


}

//ʸ������+PID����
int current_PID_FOC_coil1(double target_current,double now_current,double PWM_value) { // Ŀ���������ǰ��������ǰPWMֵ��������������PWMֵ//95,40,x    125,40,x
    double max=800,min=-800;//���������Сֵ�����ⷢ��Σ��
	  //static double kp=0,ki=10,kd=0;
	  double kp=kp_current,ki=ki_current,kd=kd_current;
    static int count=0;
    double K1=0.05;//��ֵ���ջ�������ִָ�л�������С���������޷���ͨ��������������ƾ��ȣ������Ȼ�������ڣ�ϵͳ�����Ŀ��ֵǰ��Ƶ�������������ȶ�����
    double K2=0.2;//ͨ�����ַ���ķ�ʽ��ʵ�ֿ����ֱ��ͣ����ֱ�����ִָ�л����ﵽ������������ˣ����޷�����Ŀ��ֵ���ںܳ�һ��ʱ�����޷�����������ɵġ�
    //������ֱ��͵�һ�ַ�����ʹ�û��ַ��룬�÷��������ۼ����С��ĳ����ֵ��ʹ�û�����ۼ����������ټ����ۼ����൱��ֻʹ����PD��������
    static double current_lasterror=0;
	  static double current_lastlasterror=0;
    double current_error;//�洢Ŀ������͵�ǰ�����Ĳ�ֵ
    static double current_error_accumulation=0;//�洢Ŀ������͵�ǰ�����Ĳ�ֵ֮�ͣ�������
		
	  current_lastlasterror=current_lasterror;
		current_lasterror=target_current-now_current;
	  
	  now_current=get_current1;
	  current_1=now_current; //���µ���ֵ������ֵֻ��Ҫ��PID�׶θ���
	  
    //now_current=(double)INA219_4_GetCurrent_uA()/1000/1000;  //��ȡ����������������λת��ΪA������Ӧ�÷��ڶ�ʱ����
		

    //PID_value=0;
    current_error=target_current-now_current;

    if(current_error>K1||current_error<K1*(-1)) {
        //��Ϊmotor_run�е�ѹ�ĵ�λ��0.01V����������ʽ�ж�PID�Ľ��*100      
				PWM_value+=(kp*(current_error-current_lasterror)+ki*current_error+kd*(current_error-2*current_lasterror+current_lastlasterror));
				if(PWM_value>max)
            PWM_value=max;
        if(PWM_value<min)
            PWM_value=min;

    }
		//if(voltage_amp==0)
				//PWM_value=0;
    count++;
				if(count>=5){
					//printf("%lf,%lf,%lf,%lf\n",target_current,now_current,PWM_value,current_error);
					count=0;
				}
			return PWM_value;
}
//ʸ������+PID����
int current_PID_FOC_coil2(double target_current,double now_current,double PWM_value) { // Ŀ���������ǰ��������ǰPWMֵ��������������PWMֵ//95,40,x    125,40,x
    double max=800,min=-800;//���������Сֵ�����ⷢ��Σ��
	  //static double kp=0,ki=10,kd=0;
	  double kp=kp_current,ki=ki_current,kd=kd_current;
    static int count=0;
    double K1=0.05;//��ֵ���ջ�������ִָ�л�������С���������޷���ͨ��������������ƾ��ȣ������Ȼ�������ڣ�ϵͳ�����Ŀ��ֵǰ��Ƶ�������������ȶ�����
    double K2=0.2;//ͨ�����ַ���ķ�ʽ��ʵ�ֿ����ֱ��ͣ����ֱ�����ִָ�л����ﵽ������������ˣ����޷�����Ŀ��ֵ���ںܳ�һ��ʱ�����޷�����������ɵġ�
    //������ֱ��͵�һ�ַ�����ʹ�û��ַ��룬�÷��������ۼ����С��ĳ����ֵ��ʹ�û�����ۼ����������ټ����ۼ����൱��ֻʹ����PD��������
    static double current_lasterror=0;
	  static double current_lastlasterror=0;
    double current_error;//�洢Ŀ������͵�ǰ�����Ĳ�ֵ
    static double current_error_accumulation=0;//�洢Ŀ������͵�ǰ�����Ĳ�ֵ֮�ͣ�������
		
	  current_lastlasterror=current_lasterror;
		current_lasterror=target_current-now_current;
	  
	  now_current=get_current2;
	  current_2=now_current; //���µ���ֵ������ֵֻ��Ҫ��PID�׶θ���
	  
    //now_current=(double)INA219_4_GetCurrent_uA()/1000/1000;  //��ȡ����������������λת��ΪA������Ӧ�÷��ڶ�ʱ����
		

    //PID_value=0;
    current_error=target_current-now_current;

    if(current_error>K1||current_error<K1*(-1)) {
        //��Ϊmotor_run�е�ѹ�ĵ�λ��0.01V����������ʽ�ж�PID�Ľ��*100      
				PWM_value+=(kp*(current_error-current_lasterror)+ki*current_error+kd*(current_error-2*current_lasterror+current_lastlasterror));
				if(PWM_value>max)
            PWM_value=max;
        if(PWM_value<min)
            PWM_value=min;

    }
		//if(voltage_amp==0)
				//PWM_value=0;
    count++;
				if(count>=5){
					//printf("%lf,%lf,%lf,%lf\n",target_current,now_current,PWM_value,current_error);
					count=0;
				}
			return PWM_value;
}
//ʸ������+PID����
int current_PID_FOC_coil3(double target_current,double now_current,double PWM_value) { // Ŀ���������ǰ��������ǰPWMֵ��������������PWMֵ//95,40,x    125,40,x
    double max=800,min=-800;//���������Сֵ�����ⷢ��Σ��
	  //static double kp=0,ki=10,kd=0;
	  double kp=kp_current,ki=ki_current,kd=kd_current;
    static int count=0;
    double K1=0.05;//��ֵ���ջ�������ִָ�л�������С���������޷���ͨ��������������ƾ��ȣ������Ȼ�������ڣ�ϵͳ�����Ŀ��ֵǰ��Ƶ�������������ȶ�����
    double K2=0.2;//ͨ�����ַ���ķ�ʽ��ʵ�ֿ����ֱ��ͣ����ֱ�����ִָ�л����ﵽ������������ˣ����޷�����Ŀ��ֵ���ںܳ�һ��ʱ�����޷�����������ɵġ�
    //������ֱ��͵�һ�ַ�����ʹ�û��ַ��룬�÷��������ۼ����С��ĳ����ֵ��ʹ�û�����ۼ����������ټ����ۼ����൱��ֻʹ����PD��������
    static double current_lasterror=0;
	  static double current_lastlasterror=0;
    double current_error;//�洢Ŀ������͵�ǰ�����Ĳ�ֵ
    static double current_error_accumulation=0;//�洢Ŀ������͵�ǰ�����Ĳ�ֵ֮�ͣ�������
		
	  current_lastlasterror=current_lasterror;
		current_lasterror=target_current-now_current;
	  
	  now_current=get_current3;
	  current_3=now_current; //���µ���ֵ������ֵֻ��Ҫ��PID�׶θ���
	  
    //now_current=(double)INA219_4_GetCurrent_uA()/1000/1000;  //��ȡ����������������λת��ΪA������Ӧ�÷��ڶ�ʱ����
		

    //PID_value=0;
    current_error=target_current-now_current;

    if(current_error>K1||current_error<K1*(-1)) {
        //��Ϊmotor_run�е�ѹ�ĵ�λ��0.01V����������ʽ�ж�PID�Ľ��*100      
				PWM_value+=(kp*(current_error-current_lasterror)+ki*current_error+kd*(current_error-2*current_lasterror+current_lastlasterror));
				if(PWM_value>max)
            PWM_value=max;
        if(PWM_value<min)
            PWM_value=min;

    }
		//if(voltage_amp==0)
				//PWM_value=0;
    count++;
				if(count>=5){
					//printf("%lf,%lf,%lf,%lf\n",target_current,now_current,PWM_value,current_error);
					count=0;
				}
			return PWM_value;
}
//ʸ������+PID����
int current_PID_FOC_coil4(double target_current,double now_current,double PWM_value) { // Ŀ���������ǰ��������ǰPWMֵ��������������PWMֵ//95,40,x    125,40,x
    double max=800,min=-800;//���������Сֵ�����ⷢ��Σ��
	  //static double kp=0,ki=10,kd=0;
	  double kp=kp_current,ki=ki_current,kd=kd_current;
    static int count=0;
    double K1=0.05;//��ֵ���ջ�������ִָ�л�������С���������޷���ͨ��������������ƾ��ȣ������Ȼ�������ڣ�ϵͳ�����Ŀ��ֵǰ��Ƶ�������������ȶ�����
    double K2=0.2;//ͨ�����ַ���ķ�ʽ��ʵ�ֿ����ֱ��ͣ����ֱ�����ִָ�л����ﵽ������������ˣ����޷�����Ŀ��ֵ���ںܳ�һ��ʱ�����޷�����������ɵġ�
    //������ֱ��͵�һ�ַ�����ʹ�û��ַ��룬�÷��������ۼ����С��ĳ����ֵ��ʹ�û�����ۼ����������ټ����ۼ����൱��ֻʹ����PD��������
    static double current_lasterror=0;
	  static double current_lastlasterror=0;
    double current_error;//�洢Ŀ������͵�ǰ�����Ĳ�ֵ
    static double current_error_accumulation=0;//�洢Ŀ������͵�ǰ�����Ĳ�ֵ֮�ͣ�������
		
	  current_lastlasterror=current_lasterror;
		current_lasterror=target_current-now_current;
	  
	  now_current=get_current4;
	  current_4=now_current; //���µ���ֵ������ֵֻ��Ҫ��PID�׶θ���
	  
    //now_current=(double)INA219_4_GetCurrent_uA()/1000/1000;  //��ȡ����������������λת��ΪA������Ӧ�÷��ڶ�ʱ����
		

    //PID_value=0;
    current_error=target_current-now_current;

    if(current_error>K1||current_error<K1*(-1)) {
        //��Ϊmotor_run�е�ѹ�ĵ�λ��0.01V����������ʽ�ж�PID�Ľ��*100      
				PWM_value+=(kp*(current_error-current_lasterror)+ki*current_error+kd*(current_error-2*current_lasterror+current_lastlasterror));
				if(PWM_value>max)
            PWM_value=max;
        if(PWM_value<min)
            PWM_value=min;

    }
		//if(voltage_amp==0)
				//PWM_value=0;
    count++;
				if(count>=5){
					//printf("%lf,%lf,%lf,%lf\n",target_current,now_current,PWM_value,current_error);
					count=0;
				}
			return PWM_value;
}
//��������coilΪҪ�������Ȧ��i=1Ϊ����i=0Ϊ��
void phase_convert(int coil,int i) {

    if(coil==1) {
        if(i==1) {
            GPIO_SetBits(GPIOF,GPIO_Pin_9);
            GPIO_ResetBits(GPIOF,GPIO_Pin_10);
        }

        else {
            GPIO_SetBits(GPIOF,GPIO_Pin_10);
            GPIO_ResetBits(GPIOF,GPIO_Pin_9);
        }
    }

    if(coil==2) {
        if(i==1) {
            GPIO_SetBits(GPIOF,GPIO_Pin_8);
            GPIO_ResetBits(GPIOF,GPIO_Pin_7);
        }
        else {
            
						GPIO_SetBits(GPIOF,GPIO_Pin_7);
            GPIO_ResetBits(GPIOF,GPIO_Pin_8);
        }
    }

    if(coil==3) {
        if(i==1) {
            GPIO_SetBits(GPIOA,GPIO_Pin_11);
            GPIO_ResetBits(GPIOA,GPIO_Pin_12);
        }
        else {
            GPIO_SetBits(GPIOA,GPIO_Pin_12);
            GPIO_ResetBits(GPIOA,GPIO_Pin_11);
        }
    }

    if(coil==4) {
        if(i==1) {
            GPIO_SetBits(GPIOA,GPIO_Pin_7);
            GPIO_ResetBits(GPIOA,GPIO_Pin_6);
        }
        else {
						GPIO_SetBits(GPIOA,GPIO_Pin_6);
            GPIO_ResetBits(GPIOA,GPIO_Pin_7);
            
        }
    }


}



void motor_run_DQ_new(double Q,double iq_aim,double phase) { //iq_aimΪ����Q�����
     double phase0=phase*0.01+speed*PI/25*0.02;
		double phase_add=speed*PI/25*0.02;
    double phase1=0;
    double phase2=0;
    double phase3=0;
    double phase4=0;
	  double a,b;
		double K1=200;
		double K2=0.05;
		double K3=0;
//		double R1=1;
//		double R2=1.13;
//		double R3=1.06;
//		double R4=0.96;
	 static double sum1=1;  //PID ������
		static double sum2=1;
		static double sum3=1;
		static  double sum4=1;
	
	
	
	  phase1=Q*sin(phase0+angel);
    phase2=Q*sin(phase0-phase_dif+angel);
    phase3=Q*sin(phase0-phase_dif*2+angel);
    phase4=Q*sin(phase0-phase_dif*3+angel);
		
		
		
		//PID�������費һ�´��������
		if(iq_aim!=0){
			
			if(abs(speed_filter)<3){
				sum1+=(iq_aim*0.707*sin(phase0+angel)-current_1);
				sum2+=(-1*iq_aim*0.707*sin(phase0-phase_dif+angel)-current_2);
				sum3+=(-1*iq_aim*0.707*sin(phase0-phase_dif*2+angel)-current_3);
				sum4+=(iq_aim*0.707*sin(phase0-phase_dif*3+angel)-current_4);
			}
			

			K1=50;//�ٴ���г�������
			K2=5;//�������ٴ�һ��
		  K1=kp_current;
		  K2=ki_current;
			
			
//			phase1=phase1*(1+K2*sum1);
//			phase1=phase2*(1+K2*sum2);
//			phase1=phase3*(1+K2*sum3);
//			phase1=phase4*(1+K2*sum4);
			
			phase1+=K1*(iq_aim*0.707*sin(phase0+angel)-current_1)+K2*sum1*iq_aim;
			phase2-=K1*(-1*iq_aim*0.707*sin(phase0-phase_dif+angel)-current_2)+K2*sum2*iq_aim;
			phase3-=K1*(-1*iq_aim*0.707*sin(phase0-phase_dif*2+angel)-current_3)+K2*sum3*iq_aim;
			phase4+=K1*(iq_aim*0.707*sin(phase0-phase_dif*3+angel)-current_4)+K2*sum4*iq_aim;
			}
	
	
			K3=0.8;
			//K3=kd_current;
		//�����Ȧ�еĸ�Ӧ�綯�ƽ��в���
		if(abs(speed_filter)>3){
			phase1-=K3*400.0*speed_filter*0.0026*sin(phase0+angel)/0.2711;
			phase2+=K3*400.0*speed_filter*(-1)*0.0026*sin(phase0+angel-phase_dif+0.5*PI)/0.2711;
			phase3+=K3*400.0*speed_filter*(-1)*0.0026*sin(phase0+angel-phase_dif*2)/0.2711;
			phase4-=K3*400.0*speed_filter*0.0026*sin(phase0+angel-phase_dif*3+0.5*PI)/0.2711;
		}
		

	
	//��Ȧ�ڵ���ʵ��ֵ������ֵ�Ա�
	//printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",current_1,current_2,current_3,current_4,iq_aim*0.707*sin(phase0+angel),-1*iq_aim*0.707*sin(phase0+angel-phase_dif),-1*iq_aim*0.707*sin(phase0+angel-phase_dif*2),iq_aim*0.707*sin(phase0+angel-phase_dif*3),iq);
  //printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",current_1,current_2,current_3,current_4,0.04*sin(phase0+angel),-1*0.04*sin(phase0+angel-phase_dif+0.5*PI),-1*0.04*sin(phase0+angel-phase_dif*2),0.04*sin(phase0+angel-phase_dif*3+0.5*PI),speed_filter);//�Աȸ�Ӧ�綯��
	//printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",current_1,current_2,current_3,current_4,speed_filter*0.0026*sin(phase0+angel),speed_filter*(-1)*0.0026*sin(phase0+angel-phase_dif+0.5*PI),speed_filter*(-1)*0.0026*sin(phase0+angel-phase_dif*2),speed_filter*0.0026*sin(phase0+angel-phase_dif*3+0.5*PI),iq);//�Աȸ�Ӧ�綯��
		//printf("%lf,%lf,%lf,%lf\n",current_1,iq_aim*0.707*sin(phase0+angel),phase1,iq);
	
    if(phase1>=0) {
        TIM_SetCompare1(TIM2, PWM_T-phase1);
        phase_convert(1,1);
    }
    else {
        phase1=abs(phase1);
        TIM_SetCompare1(TIM2, PWM_T-phase1);
        phase_convert(1,-1);
    }

    if(phase2>=0) {
        TIM_SetCompare2(TIM2, PWM_T-phase2);
        phase_convert(2,1);
    }
    else {
        phase2=abs(phase2);
        TIM_SetCompare2(TIM2, PWM_T-phase2);
        phase_convert(2,-1);
    }
    if(phase3>=0) {
        TIM_SetCompare3(TIM3, PWM_T-phase3);
        phase_convert(3,1);
    }
    else {
        phase3=abs(phase3);
        TIM_SetCompare3(TIM3, PWM_T-phase3);
        phase_convert(3,-1);
    }
    if(phase4>=0) {
        TIM_SetCompare4(TIM3, PWM_T-phase4);
        phase_convert(4,1);
    }
    else {
        phase4=abs(phase4);
        TIM_SetCompare4(TIM3, PWM_T-phase4);
        phase_convert(4,-1);
    }
}
//�����˶���ampΪ���ҵ�ѹ��ֵ,��λΪ0.01V��phaseΪ��λ
void motor_run(int amp,int phase) {
	
	  //  �ĸ���Ȧ�ĵ��裬�ⶨ
		double R1=1;
		double R2=1.13;
		double R3=1.06;
		double R4=0.96;
		
    double phase0=phase*0.01+speed*PI/25*0.02;

    
    int phase1=amp*sin(phase0+angel)*R1;
    int phase2=amp*sin(phase0-phase_dif+angel)*R2 ;
    int phase3=amp*sin(phase0-phase_dif*2+angel)*R3 ;
    int phase4=amp*sin(phase0-phase_dif*3+angel)*R4 ;

    
    //����֣���λ����ܷ�
//    int phase1=amp*sin(phase0+angel);
//    int phase2=amp*sin(phase0+phase_dif+angel);
//    int phase3=amp*sin(phase0+phase_dif*2+angel);
//    int phase4=amp*sin(phase0+phase_dif*3+angel);
		

		//printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",current_1,current_2,current_3,current_4,id,iq,speed*PI/25,phase/100+angel);
	
	  Four_phase_clark(phase1,phase2,phase3,phase4,&ualphi,&ubeta);
		Park(ualphi,ubeta,phase/100.0+angel,&ud,&uq);


    //����ÿһ����Ȧ�ĵ�ѹ�������÷���
    //printf("%d,%d,%d,%d\n",phase1,phase2,phase3,phase4);
    if(phase1>=0) {
        TIM_SetCompare1(TIM2, PWM_T-phase1);
        phase_convert(1,1);

      }
    else {
        phase1=abs(phase1);
        TIM_SetCompare1(TIM2, PWM_T-phase1);
        phase_convert(1,-1);
    }

    if(phase2>=0) {
        TIM_SetCompare2(TIM2, PWM_T-phase2);
        phase_convert(2,1);
    }
    else {
        phase2=abs(phase2);
        TIM_SetCompare2(TIM2, PWM_T-phase2);
        phase_convert(2,-1);
    }

    if(phase3>=0) {
        TIM_SetCompare3(TIM3, PWM_T-phase3);
        phase_convert(3,1);
    }
    else {
        phase3=abs(phase3);
        TIM_SetCompare3(TIM3, PWM_T-phase3);
        phase_convert(3,-1);
    }
    if(phase4>=0) {
        TIM_SetCompare4(TIM3, PWM_T-phase4);
        phase_convert(4,1);
    }
    else {
        phase4=abs(phase4);
        TIM_SetCompare4(TIM3, PWM_T-phase4);
        phase_convert(4,-1);
    }

}
//����������ʹ��ÿ����Ȧ�ڵ����ӽ��ο�ֵ��
void motor_run_new(int amp,int phase) {
	
	  //  �ĸ���Ȧ�ĵ���
		double R1=1/0.80;
		double R2=1/0.774;
		double R3=1/0.686;
		double R4=1/0.673;
		double K1=400;
		
    double phase0=phase*0.01+speed*PI/25*0.02;

    /*
    int phase1=amp*sin(phase0+angel);
    int phase2=amp*sin(phase0-phase_dif+angel);
    int phase3=amp*sin(phase0-phase_dif*2+angel);
    int phase4=amp*sin(phase0-phase_dif*3+angel);
    */
    //����֣���λ����ܷ�
    double phase1=R1*amp*sin(phase0+angel);
    double phase2=R2*amp*sin(phase0+phase_dif+angel);
    double phase3=R3*amp*sin(phase0+phase_dif*2+angel);
    double phase4=R4*amp*sin(phase0+phase_dif*3+angel);
		

		
	
	  

		//printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",current_1,current_2,current_3,current_4,id,iq,speed*PI/25,phase/100+angel);
		//printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",current_1,current_2,current_3,current_4,amp*sin(phase0+angel)/400.0,K1*(phase1/400.0-current_1)/sin(phase*0.01+angel)*sin(phase0+angel),phase3/400.0,phase4/400.0);
	
	phase1+=K1*(amp*sin(phase0+angel)/400.0-current_1);
	phase2+=K1*(amp*sin(phase0+angel+phase_dif)/400.0-current_2);
	phase3+=K1*(amp*sin(phase0+angel+phase_dif*2)/400.0-current_3);
	phase4+=K1*(amp*sin(phase0+angel+phase_dif*3)/400.0-current_4);

	
	Four_phase_clark(phase1,phase2,phase3,phase4,&ualphi,&ubeta);
		Park(ualphi,ubeta,phase/100.0+angel,&ud,&uq);
    //����ÿһ����Ȧ�ĵ�ѹ�������÷���
    //printf("%d,%d,%d,%d\n",phase1,phase2,phase3,phase4);
    if(phase1>=0) {
        TIM_SetCompare1(TIM2, PWM_T-phase1);
        phase_convert(1,1);

      }
    else {
        phase1=abs(phase1);
        TIM_SetCompare1(TIM2, PWM_T-phase1);
        phase_convert(1,-1);
    }

    if(phase2>=0) {
        TIM_SetCompare2(TIM2, PWM_T-phase2);
        phase_convert(2,1);
    }
    else {
        phase2=abs(phase2);
        TIM_SetCompare2(TIM2, PWM_T-phase2);
        phase_convert(2,-1);
    }

    if(phase3>=0) {
        TIM_SetCompare3(TIM3, PWM_T-phase3);
        phase_convert(3,1);
    }
    else {
        phase3=abs(phase3);
        TIM_SetCompare3(TIM3, PWM_T-phase3);
        phase_convert(3,-1);
    }
    if(phase4>=0) {
        TIM_SetCompare4(TIM3, PWM_T-phase4);
        phase_convert(4,1);
    }
    else {
        phase4=abs(phase4);
        TIM_SetCompare4(TIM3, PWM_T-phase4);
        phase_convert(4,-1);
    }

}
//DQ����ƣ�DΪD���ѹ��QΪQ���ѹ,phaseΪ��λ*100
void motor_run_DQ(double D,double Q,double phase) {
    double phase0=phase*0.01+speed*PI/25*0.02;
		double phase_add=speed*PI/25*0.02;
    double phase1=0;
    double phase2=0;
    double phase3=0;
    double phase4=0;
	  double a,b;
		double error1,error2,error3,error4;
	  Park_inverse( D, Q, phase/100.0+angel+speed*PI/25*0.02, &a, &b);//�����ӳٲ���
		Four_phase_clark_inverse( a, b, &phase1, &phase2, &phase3, &phase4);
		phase3=phase3*1.17;//�����������
		phase4=phase4*1.17;
		
		error1=2.0/3.0*sin(phase*0.01+angel);
		error2=sin(phase*0.01+0.75*PI*1);
		error3=sin(phase*0.01+0.75*PI*2);
		error4=sin(phase*0.01+0.75*PI*3);
	//printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",current_1,current_2,current_3,current_4,error1,error2,error3,error4);
	//printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",phase1/400,phase2/400,phase3/400,phase4/400,error1,error2,error3,error4);
	//printf("%lf,%lf,%lf,%lf\n",current_1,0.1*sin(phase*0.01+speed*PI/25*0.02+angel),0.1*sin(phase*0.01+angel),speed);
	
	
//	phase1=Q*sin(phase0);
//	phase2=Q*sin(phase0+0.75*PI*1);
//	phase3=Q*sin(phase0+0.75*PI*2);
//	phase4=Q*sin(phase0+0.75*PI*3);
//	error1=(phase1/400-current_1)/sin(phase0);
//	phase1=Q/2*sin(phase0+phase_add)+error1*sin(phase0+phase_add);
//	error2=(phase2/400-current_2)/sin(phase0+0.75*PI*1);
//	phase2=Q/2*sin(phase0+phase_add+0.75*PI*1)+error2*sin(phase0+phase_add+0.75*PI*1);
//	error3=(phase3/400-current_3)/sin(phase0+0.75*PI*2);
//	phase3=Q/2*sin(phase0+phase_add+0.75*PI*2)+error3*sin(phase0+phase_add+0.75*PI*2);
//	error4=(phase4/400-current_4)/sin(phase0+0.75*PI*3);
//	phase4=Q/2*sin(phase0+phase_add+0.75*PI*3)+error4*sin(phase0+phase_add+0.75*PI*3);
    if(phase1>=0) {
        TIM_SetCompare1(TIM2, PWM_T-phase1);
        phase_convert(1,1);
    }
    else {
        phase1=abs(phase1);
        TIM_SetCompare1(TIM2, PWM_T-phase1);
        phase_convert(1,-1);
    }

    if(phase2>=0) {
        TIM_SetCompare2(TIM2, PWM_T-phase2);
        phase_convert(2,1);
    }
    else {
        phase2=abs(phase2);
        TIM_SetCompare2(TIM2, PWM_T-phase2);
        phase_convert(2,-1);
    }

    if(phase3>=0) {
        TIM_SetCompare3(TIM3, PWM_T-phase3);
        phase_convert(3,1);
    }
    else {
        phase3=abs(phase3);
        TIM_SetCompare3(TIM3, PWM_T-phase3);
        phase_convert(3,-1);
    }
    if(phase4>=0) {
        TIM_SetCompare4(TIM3, PWM_T-phase4);
        phase_convert(4,1);
    }
    else {
        phase4=abs(phase4);
        TIM_SetCompare4(TIM3, PWM_T-phase4);
        phase_convert(4,-1);
    }

}//DQ����ƣ�DΪD���ѹ��QΪQ���ѹ,phaseΪ��λ*100
void motor_run_DQ_2(double D,double Q,double phase) {
    double phase0=phase*0.01;
		
    int phase1=Q*sin(phase0+angel);
    int phase2=Q*sin(phase0+phase_dif+angel);
    int phase3=Q*sin(phase0+phase_dif*2+angel);
    int phase4=Q*sin(phase0+phase_dif*3+angel);
	 
		

    //����ÿһ����Ȧ�ĵ�ѹ�������÷���
    //printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,",phase,D,Q,phase1,phase2,phase3,phase4);
    if(phase1>=0) {
        TIM_SetCompare1(TIM2, PWM_T-phase1);
        phase_convert(1,1);
    }
    else {
        phase1=abs(phase1);
        TIM_SetCompare1(TIM2, PWM_T-phase1);
        phase_convert(1,-1);
    }

    if(phase2>=0) {
        TIM_SetCompare2(TIM2, PWM_T-phase2);
        phase_convert(2,1);
    }
    else {
        phase2=abs(phase2);
        TIM_SetCompare2(TIM2, PWM_T-phase2);
        phase_convert(2,-1);
    }

    if(phase3>=0) {
        TIM_SetCompare3(TIM3, PWM_T-phase3);
        phase_convert(3,1);
    }
    else {
        phase3=abs(phase3);
        TIM_SetCompare3(TIM3, PWM_T-phase3);
        phase_convert(3,-1);
    }
    if(phase4>=0) {
        TIM_SetCompare4(TIM3, PWM_T-phase4);
        phase_convert(4,1);
    }
    else {
        phase4=abs(phase4);
        TIM_SetCompare4(TIM3, PWM_T-phase4);
        phase_convert(4,-1);
    }

}
//�����˶���ampΪ���ҵ�ѹ��ֵ,��λΪ0.01V��phaseΪ��λ
//���FOC
void motor_run_FOC(double amp,int phase) {
  double phase0=phase*0.01;


    //��λ����ܷ�
	  static int PWM_value1=0;
		static int PWM_value2=0;
		static int PWM_value3=0;
		static int PWM_value4=0;
		double target_current_1=amp*sin(phase0+angel);  
		double target_current_2=amp*sin(phase0+phase_dif+angel);
		double target_current_3=amp*sin(phase0+phase_dif*2+angel);
		double target_current_4=amp*sin(phase0+phase_dif*3+angel);

		//current_1=get_current1;
		//current_2=get_current2;
		//current_3=get_current3;
		//current_4=get_current4;
		
	
	
			
		
		//ʹ��PiDʹ�õ�������Ŀ�����
		PWM_value1=current_PID_FOC_coil1(target_current_1,current_1,PWM_value1);	
		PWM_value2=current_PID_FOC_coil2(target_current_2,current_2,PWM_value2);
		PWM_value3=current_PID_FOC_coil3(target_current_3,current_3,PWM_value3);
		PWM_value4=current_PID_FOC_coil4(target_current_4,current_4,PWM_value4);	
		//printf("%lf,%lf\n",target_current_4,current_4);
		printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",target_current_1,current_1,target_current_2,current_2,target_current_3,current_3,target_current_4,current_4);
		//printf("%lf,%lf,%lf,%lf,%d,%d,%d,%d\n",(double)ina219_GetCurrent_uA()/1000/1000,(double)INA219_2_GetCurrent_uA()/1000/1000,(double)INA219_3_GetCurrent_uA()/1000/1000,(double)INA219_4_GetCurrent_uA()/1000/1000,PWM_value1,PWM_value2,PWM_value3,PWM_value4);
		//printf("%d,%lf,%lf,%d\n",amp,target_current_1,(double)ina219_GetCurrent_uA()/1000/1000,PWM_value1);	


    //����ÿһ����Ȧ�ĵ�ѹ�������÷���
    //printf("%d,%d,%d,%d\n",phase1,phase2,phase3,phase4);
    if(PWM_value1>0) {
        TIM_SetCompare1(TIM2, PWM_T-PWM_value1);
        phase_convert(1,1);
    }
    else {
           //�д�����
        TIM_SetCompare1(TIM2, PWM_T-abs(PWM_value1));
        phase_convert(1,-1);
    }

    if(PWM_value2>=0) {
        TIM_SetCompare2(TIM2, PWM_T-PWM_value2);
        phase_convert(2,1);
    }
    else {
        
        TIM_SetCompare2(TIM2, PWM_T-abs(PWM_value2));
        phase_convert(2,-1);
    }

    if(PWM_value3>=0) {
        TIM_SetCompare3(TIM3, PWM_T-PWM_value3);
        phase_convert(3,1);
    }
    else {
       
        TIM_SetCompare3(TIM3, PWM_T-abs(PWM_value3));
        phase_convert(3,-1);
    }
    if(PWM_value4>=0) {
        TIM_SetCompare4(TIM3, PWM_T-PWM_value4);
        phase_convert(4,1);
    }
    else {
        
        TIM_SetCompare4(TIM3, PWM_T-abs(PWM_value4));
        phase_convert(4,-1);
    }

}
//���ڳ�ʼ��

/*void GPIOInit() {  //��ʼ��PB3,PB4,PD1,PD2
    GPIO_InitTypeDef  GPIO_InitStructure;

	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);	 //ʹ��PF�˿�ʱ��
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	  GPIO_InitStructure.GPIO_Pin = GPIOF_PINS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOF, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOF.78910
    GPIO_ResetBits(GPIOF,GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10);						 // �����
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PA�˿�ʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_11|GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.34
    GPIO_ResetBits(GPIOA,GPIO_Pin_12|GPIO_Pin_11|GPIO_Pin_6|GPIO_Pin_7);						 // �����

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PB�˿�ʱ��
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
    GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.34
    GPIO_SetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_11);						 // �����



}*/
/*#define GPIOF_PINS (GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10)
#define GPIOA_PINS (GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_6 | GPIO_Pin_7)
#define GPIOB_PINS (GPIO_Pin_12 | GPIO_Pin_11)
#define GPIO_PORTF GPIOF
#define GPIO_PORTA GPIOA
#define GPIO_PORTB GPIOB
#define GPIO_MODE_OUTPUTF GPIO_Mode_Out_PP
#define GPIO_MODE_OUTPUTA GPIO_Mode_Out_PP
#define GPIO_MODE_OUTPUTB GPIO_Mode_Out_PP
#define GPIO_SPEEDF GPIO_Speed_50MHz
#define GPIO_SPEEDA GPIO_Speed_50MHz
#define GPIO_SPEEDB GPIO_Speed_50MHz

void GPIO_init_function(uint32_t GPIOX,uint16_t GPIOX_PINS, uint32_t RCC_APB2Periph_GPIOX, uint32_t GPIO_SPEED, uint32_t GPIO_MODE_OUTPUT ){
    
	  GPIO_InitTypeDef  GPIO_InitStructure;
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOX, ENABLE);	 //ʹ��PX�˿�ʱ��
    GPIO_InitStructure.GPIO_Pin = GPIOX_PINS;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT; 		 //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_SPEED;		 //IO���ٶ�
    GPIO_Init(GPIOX, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOF.78910
    GPIO_ResetBits(GPIOX,GPIOX_PINS);						 // �����

}
	  
void GPIOInit() {  
    //
    GPIO_init_function(GPIO_PORTF,GPIOF_PINS, RCC_APB2Periph_GPIOF, GPIO_SPEEDF, GPIO_MODE_OUTPUTF );
    GPIO_init_function(GPIO_PORTA,GPIOA_PINS, RCC_APB2Periph_GPIOA, GPIO_SPEEDA, GPIO_MODE_OUTPUTA );
	  GPIO_init_function(GPIO_PORTB,GPIOB_PINS, RCC_APB2Periph_GPIOB, GPIO_SPEEDB, GPIO_MODE_OUTPUTB );


}*/
// ����GPIO����
/*#define GPIOF_PINS (GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10)
#define GPIOA_PINS (GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_6 | GPIO_Pin_7)
#define GPIOB_PINS (GPIO_Pin_12 | GPIO_Pin_11)

// ����GPIO�˿�
#define GPIO_PORTF GPIOF
#define GPIO_PORTA GPIOA
#define GPIO_PORTB GPIOB

// ����GPIOģʽ�ٶ�
#define GPIO_MODE_OUTPUT GPIO_Mode_Out_PP
#define GPIO_SPEED GPIO_Speed_50MHz*/

// GPIO ��ʼ��
void GPIO_init_function(GPIO_TypeDef * GPIOX, uint16_t GPIOX_PINS, uint32_t RCC_APB2Periph_GPIOX, GPIO_InitTypeDef *GPIO_InitStructure) {
 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOX, ENABLE);  // ʹ��PX�˿�ʱ��
    GPIO_InitStructure->GPIO_Pin = GPIOX_PINS;               
    GPIO_InitStructure->GPIO_Mode = GPIO_MODE_OUTPUT;        
    GPIO_InitStructure->GPIO_Speed = GPIO_SPEED;       
    GPIO_Init(GPIOX, GPIO_InitStructure);                  
    GPIO_ResetBits(GPIOX, GPIOX_PINS);                      
}

// GPIO ��ʼ���ܺ���
void GPIOInit() {  
	  GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_init_function((GPIO_TypeDef *)GPIO_PORTF, GPIOF_PINS, RCC_APB2Periph_GPIOF, &GPIO_InitStructure);
    GPIO_init_function((GPIO_TypeDef *)GPIO_PORTA, GPIOA_PINS, RCC_APB2Periph_GPIOA, &GPIO_InitStructure);
    GPIO_init_function((GPIO_TypeDef *)GPIO_PORTB, GPIOB_PINS, RCC_APB2Periph_GPIOB, &GPIO_InitStructure);
}

//���ݻ����������źŻ�ȡ��ǰλ��
void get_position() {
		static int count=0;
    double speed_row=0;
		double AN_hall1,AN_hall2;
    
		static double gap_time=0;
    //�Ի����������źŹ�һ��
    //hall1=(float)Get_Adc_Average(14,3)*(3.3/4096);
    //hall2=(float)Get_Adc_Average(15,3)*(3.3/4096);
    hall1=(float)Get_Adc_Average(14,5)*(3.3/4096);
    hall2=(float)Get_Adc_Average(15,5)*(3.3/4096);
		
		hall1=hall1/3.26;
	hall2=hall2/4.15;
	 // hall1=hall1/4.2/0.769;
		//hall2=hall2/3.177/1.317;
    hall1=hall1-0.5;
    hall2=hall2-0.5;
    

    hall2=hall2/0.778;
    if(hall1<0.01&&hall1>-0.01) {  //��ֹ�����쳣״��
        if(hall1>0)
            hall1=0.005;
        else
            hall1=-0.005;
    }
	
    phase=atan(hall1/hall2)*100;
		//printf("%lf,%lf,%lf\n",hall1,hall2,iq);

    if(hall1>0&&hall2>0)   //��360�ȷֳ�4����ͬ������
        phase=phase+PI*50;
    else if(hall1>0&&hall2<0)
        phase=phase+PI*150;
    else if(hall1<0&&hall2<0)
        phase=phase+PI*150;
    else if(hall1<0&&hall2>0)
        phase=phase+PI*50;





    if(abs(phase-last_phase)<100)              //���Ǳ߽����
        position=position+(phase-last_phase)/PI/200*T;
    else if(phase<last_phase)
        position=position+(phase+PI*200-last_phase)/PI/200*T;
    else
        //position=position+(-phase+(last_phase+PI*200))/PI/200*T;
        //position=position-(phase-PI*200-last_phase)/PI/200*T;
				position=position-(-phase+(last_phase+PI*200))/PI/200*T;
    last_phase=phase;
		

		count++;
		gap_time=gap_time+runTime;
		if(count>=1){
			filter_U.In=(position-last_position)/(gap_time/1000000);
			filter_step();
			speed_filter=filter_Y.speed;  //��ȡ�˲�֮����ٶ�
			acceleration_filter=filter_Y.acc;
			
			gap_time=0.02*1000000;//�̶�����Ƶ��
			speed=(position-last_position)/(gap_time/1000000);  //�˲����� �ٶ����ӳ�
			
			acceleration=(speed-last_speed)/(gap_time/1000000);
			//printf("%lf,%lf\n",speed,(position-last_position)/(gap_time/1000000));
			last_position=position;
			last_speed=speed;
			last_speed_filter=speed_filter;
			count=0;
			gap_time=0;
		}    
    //speed_row=(position-last_position)/difftime(clock(),c_start)/1000;
    //push(&Speed, speed_row);
    //c_start = clock();
    //avg(&Speed, &speed);
}




//������������ʽPID����
void current_PID(double target_current) { //95,40,x    125,40,x
    double max=800,min=-800;//���������Сֵ�����ⷢ��Σ��
static int count=0;
    double K1=0.003;//��ֵ���ջ�������ִָ�л�������С���������޷���ͨ��������������ƾ��ȣ������Ȼ�������ڣ�ϵͳ�����Ŀ��ֵǰ��Ƶ�������������ȶ�����
    double K2=0.2;//ͨ�����ַ���ķ�ʽ��ʵ�ֿ����ֱ��ͣ����ֱ�����ִָ�л����ﵽ������������ˣ����޷�����Ŀ��ֵ���ںܳ�һ��ʱ�����޷�����������ɵġ�
    //������ֱ��͵�һ�ַ�����ʹ�û��ַ��룬�÷��������ۼ����С��ĳ����ֵ��ʹ�û�����ۼ����������ټ����ۼ����൱��ֻʹ����PD��������
    static double current_lasterror=0;
	  static double current=0;
	  static double current_lastlasterror=0;
    double current_error;//�洢Ŀ������͵�ǰ�����Ĳ�ֵ
    static double current_error_accumulation=0;//�洢Ŀ������͵�ǰ�����Ĳ�ֵ֮�ͣ�������
		int current_zheng=1; //��ǰ����Ϊ��
		
	  current_lastlasterror=current_lasterror;
		current_lasterror=target_current-current;
		current=iq;
    //current=(double)ina219_GetCurrent_uA()/1000/1000;  //��ȡ����������������λת��ΪA������Ӧ�÷��ڶ�ʱ����
		//current=(double)ina226_GetCurrent_uA()/1000/1000;
		

    //PID_value=0;
    current_error=target_current-current;
    //current_error_accumulation=current_error_accumulation+current_error;
	  //push(&current_errors,current_error);
		//sum(&current_errors,&current_error_accumulation);
    //if(current_error>K2||current_error<K2*(-1)||(current_error>0.002&&current_error<0.002*(-1)))
        //current_error_accumulation=0;
    if(current_error>K1||current_error<K1*(-1)) {
        //��Ϊmotor_run�е�ѹ�ĵ�λ��0.01V����������ʽ�ж�PID�Ľ��*100
        //PID_value+=(kp_current*current_error+ki_current*current_error_accumulation+kd_current*(current_error-current_lasterror));
				PID_value+=(kp_current*(current_error-current_lasterror)+ki_current*current_error+kd_current*(current_error-2*current_lasterror+current_lastlasterror));
				if(PID_value>max)
            PID_value=max;
        if(PID_value<min)
            PID_value=min;

    }
		if(voltage_amp==0)
				PID_value=0;
    count++;
				if(count>=3){
					//printf("%lf,%lf\n",position,current);
					//printf("%lf,%lf,%lf\n",target_current,current,PID_value);
					count=0;
				}
}

//��������ѹת��Ϊ��Ӧ����
double voltage2current(double voltage) {
		if(voltage<0){
			voltage=voltage*(-1);		
			return (0.30089*voltage*voltage -0.001*voltage + 0.0663)*(-1);
		}			
    return 0.30089*voltage*voltage -0.001*voltage + 0.0663;
}
//����λ�����ݵ�ǰ�����״̬��Ϣ
void status_information() {
		//printf("%lf,%lf\n",position,current);
    if(get_hall==1) {
        printf("pos %lf\n",position);
			
        //printf("%lf,%lf,%lf,%lf\n",hall1,hall2,phase/100.0,position);
    }
    if(get_cur==1) {
        printf("cur %lf\n",current);
        //printf("curent %lf voltage_amp+PID_value %d\n",current,voltage_amp+PID_value);
    }

}
int main(void)
{
    //int voltage_source=10;//��Դ��ѹ
    //int PWM_T=voltage_source*100-1;//PWM����//�˱���ӦΪȫ�ֱ���
	  //int PWM_T=4000-1;//PWM����
    u16 t;
    int recive_num=0;
    u16 len;
    u16 led0pwmval=0;
    u8 dir=1;
		static int count=0;
		double D_target=0;
		double Q_target=0;
		double last_iq=0;
		double last_id=0;
	  char placeholder[120];

    char cmd[50]= {' '}; //�洢���յ�������
    char *cmd_split;
    delay_init();	    	 //��ʱ������ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
    uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
    GPIOInit();
    Adc_Init();
		MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);//DMA1ͨ��4,����Ϊ����1,�洢��ΪSendBuff,����SEND_BUF_SIZE.


    TIM3_Int_Init(PWM_T,0);//�����ж�
		//TIM2_Int_Init(9999,7200-1);

    servo_init(PWM_T,0);
		//servo_init(PWM_T,3);//�����Ƶϵ���ɼ��ٵ���������г��   72000/1000=72KHz
    TIM_SetCompare1(TIM2, PWM_T-1);
    TIM_SetCompare2(TIM2, PWM_T-1);
    TIM_SetCompare3(TIM3, PWM_T-1);
    TIM_SetCompare4(TIM3, PWM_T-1);
    delay_ms(100);
    c_start = clock();    //!< ��λΪms

		//ina219_init();

		INA3221_Init(0x80);
		INA3221_2_Init(0x80);//new
		
   filter_initialize();//�˲�����ʼ��
	 parameter_identify_initialize();//������ʶ��ʼ��
	 FluxObserver_initialize();//�����۲�����ʼ��
	 SMO_initialize();//��Ĥ�۲�����ʼ��

	
		get_position();
    light_ruler_position=position;
    //motor_run(100,0);//�ǲ���Ӧ��ɾ��
    t=0;
		MeasureTimeStart();
		print("start");
    while(1)
    {
      get_position(); //6%
        
			runTime = MeasureTimeStop(72); //�������ʱ��,��λ΢��
			MeasureTimeStart();
			

			//motor_run(10.0/voltage_source*(voltage_amp),phase);
			//motor_run_new(10.0/voltage_source*(voltage_amp),phase); //15%
			
			
			
			current_1=get_current1;  //32%
			current_2=get_current2;
			current_3=get_current3;
			current_4=get_current4;
			force_observer();
				
			Four_phase_clark(current_1,current_2,current_3,current_4,&ialphi,&ibeta);  //7%
			//Park(ialphi,ibeta,phase/100.0+angel+0.25*3.14,&id,&iq);
			Park(ialphi,ibeta,phase/100.0+angel+0.25*3.14,&iq,&id);//id��iq����
			//clark_and_park(current_1,current_2,current_3,current_4,0,&ialphi,&ibeta);
			//printf("%lf,%lf,%lf,%lf,%lf\n",ialphi,ibeta,id,iq,phase/100.0+angel);
			iq=medianFilter3_1(iq);
			
			
			//�˳�λ�ý�Ծ����µĳ������
//		if(iq<0)   //����BUG�������iqֻ������������ֵ��Ϊ0
//			{
//				iq=last_iq;
//				id=last_id;
//			}
//			last_iq=iq;
//			last_id=id;
						
			//Q�����PID����
			PID_value_force=force_PID(acceleration_filter,speed_filter);
			//PID_value_Q=Q_current_PID(voltage_amp/100.0-PID_value_force,iq,PID_value_Q); //�Ӵ�������
			PID_value_Q=0;
			PID_value_Q=pidController(voltage_amp/100.0-iq);
			//PID_value_Q=Q_current_PID(voltage_amp/100.0,iq,PID_value_Q);
			PID_value_Q=medianFilter3_2(PID_value_Q); //��ֵ�˲�
			//PID_value_D=D_current_PID(0,id,PID_value_D);	
			
			
			//MPC����Ԥ�����
			//MRAC(0,10.0/voltage_source*(voltage_amp/100.0)-PID_value_force,id,iq,speed_filter*PI/25,&D_target,&Q_target);//�Ӵ�������
			MRAC(0,10.0/voltage_source*(voltage_amp/100.0),id,iq,speed_filter*PI/25,&D_target,&Q_target);//��ʱvoltage_amp�ĵ�λ��0.01v
			
			//MRAC(0,tri_wave(),id,iq,speed_filter*PI/25,&D_target,&Q_target);//���ǲ�����
			
			//Q_target=Q_target+PID_value_Q;  //������PID��MPC�ֿ���˳��ִ�е�Ч��
			//D_target=D_target+PID_value_D;
			if(Q_target<0)//��ֹ������
				Q_target=0;
			
			motor_run_DQ_new(Q_target,10.0/voltage_source*(voltage_amp/100.0),phase);//��Ȧ�ڵ����ӳٲ���
			//motor_run(Q_target,phase);
			

			
			//printf("%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf,%.3lf\n",10.0/voltage_source*(voltage_amp/100.0)-iq,0-id,iq,id,Q_target/400.0,D_target/400.0,speed*PI/25); //50%
			if(DMA_GetFlagStatus(DMA1_FLAG_TC4)!=RESET)	//�ж�ͨ��4�������
				{	
					//sprintf(placeholder,"%lf,%lf,%lf,%lf\n",iq,id,speed_filter*PI/25,Q_target/400);
					//���������ַ�����ֵ��placeholder��
					sprintf(placeholder,"%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",iq,acceleration_filter,speed_filter,force_e,force_contact,Q_target,PID_value_Q);
					//sprintf(placeholder,"%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",current_1,current_2,current_3,current_4,iq,speed_filter,Q_target);
					//sprintf(placeholder,"%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",current_1,current_2,current_3,current_4,sin(phase/100.0+angel+phase_dif*0),sin(phase/100.0+angel+phase_dif*1),sin(phase/100.0+angel+phase_dif*2),sin(phase/100.0+angel+phase_dif*3));
					DMA_ClearFlag(DMA1_FLAG_TC4);//���ͨ��4������ɱ�־
					print(placeholder);
		    }
			

			
			
		
			count++;
				if(count>=50*2.5){
//					if(angel==0.6)
//						angel=0.6+3.14;  //��Ҫ����һ�������ǲ���3.14
//					else angel=0.6;
					

					count=0;
				}

        //status_information();
    }
}
//��ʱ��2�жϷ������, ÿ��100hz
void TIM2_IRQHandler(void){
	static int count=0;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ
    {        
				//printf("TIM2_IRQHandler\n");
			count++;
				if(count>=2000){
					status_information();
					count=0;
				}
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ  
		}
}
//��ʱ��3�жϷ������, ÿ��25600hz
void TIM3_IRQHandler(void)   //TIM3�ж�
{
		static int count=0;
    u16 len;
		double D_target=0;
		double Q_target=0;
    char cmd[50]= {' '}; //�洢���յ�������
    char *cmd_split;
    int recive_num=0;
    int t;
		int i=0;
    static int t_times1=0;//TIM3�ж�ִ�еĴ���
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ
        
        if(USART_RX_STA&0x8000)
        {																			
            len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
            memset(cmd,' ',sizeof(cmd)/sizeof(cmd[0]));//��cmd����

            //memcpy(cmd,USART_RX_BUF,sizeof(USART_RX_BUF)/sizeof(USART_RX_BUF[0])); //�ַ������ƣ�cmd����string�ĺ�������
            memcpy(cmd,USART_RX_BUF,len); //�ַ������ƣ�cmd����string�ĺ�������

            cmd_split=strtok(cmd," ");//�ַ����ָ�
            //printf("%s",cmd_split);

            if(memcmp(cmd_split,"system",sizeof(cmd_split)/sizeof(cmd_split[0]))==0) { //change voltage_source
                cmd_split = strtok(NULL," ");

                printf("voltage_source is %d\n",voltage_source);
                printf("voltage_amp is %d\n",voltage_amp);
                printf("PWM_T is %d\n",PWM_T);
                printf("angel is %lf\n",angel);
                printf("phase_dif is %lf\n",phase_dif);
                printf("phase is %lf\n",phase);

                printf("phase1 is %lf\n",voltage_amp*sin(phase*0.1+angel));
                printf("phase2 is %lf\n",voltage_amp*sin(phase*0.1-phase_dif+angel));
                printf("phase3 is %lf\n",voltage_amp*sin(phase*0.1-phase_dif*2+angel));
                printf("phase4 is %lf\n",voltage_amp*sin(phase*0.1-phase_dif*3+angel));

                USART_RX_STA=0;
                return;
            }

            if(memcmp(cmd_split,"vos",sizeof(cmd_split)/sizeof(cmd_split[0]))==0) { //change voltage_source
                cmd_split = strtok(NULL," ");
                recive_num=0;
                for(t=0; t<get_length(cmd_split); t++)
                {
                    recive_num=cmd_split[t]-'0'+recive_num*10;
                }
                printf("change vos %d\n",recive_num);
                voltage_source=recive_num;//��Դ��ѹ
                //PWM_T=voltage_source*100-1;//PWM����
                USART_RX_STA=0;
                return;
            }

            if(memcmp(cmd_split,"vol",sizeof(cmd_split)/sizeof(cmd_split[0]))==0) {
                cmd_split = strtok(NULL," ");
                recive_num=0;
							if(cmd_split[0]!='-')
                for(t=0; t<get_length(cmd_split); t++)
                {
                    recive_num=cmd_split[t]-'0'+recive_num*10;
                }
							/*else
								for(t=1; t<get_length(cmd_split); t++)
                {
                    recive_num=-cmd_split[t]+'0'+recive_num*10;
                }
								*/
                //
								//printf("change vol %d\n",recive_num);
                //TIM_SetCompare2(TIM3,PWM_T-recive_num);	//���ñȽ���
                voltage_amp=recive_num;
                //motor_run(recive_num,phase);
								
//								if(10.0/voltage_source*(voltage_amp/100.0)-iq>0.5){
//									for(i=1;i<(10.0/voltage_source*(voltage_amp/100.0)-iq)/0.5;i++){
//										MRAC(0,iq+i*0.5,id,iq,speed*PI/25,&D_target,&Q_target);//��ʱvoltage_amp�ĵ�λ��0.01v			
//										motor_run_DQ_new(0,Q_target,10.0/voltage_source*(voltage_amp/100.0),phase);//��Ȧ�ڵ����ӳٲ���
//										delay_ms(1);
//									}
//								}
//								else if(10.0/voltage_source*(voltage_amp/100.0)-iq<-0.5){
//									for(i=1;i<(iq-10.0/voltage_source*(voltage_amp/100.0))/0.5-1;i++){
//										MRAC(0,iq-i*0.5,id,iq,speed*PI/25,&D_target,&Q_target);//��ʱvoltage_amp�ĵ�λ��0.01v			
//										motor_run_DQ_new(0,Q_target,10.0/voltage_source*(voltage_amp/100.0),phase);//��Ȧ�ڵ����ӳٲ���
//										delay_ms(1);
//									}
//								}
////								//MPC����Ԥ�����
//								MRAC(0,10.0/voltage_source*(voltage_amp/100.0),id,iq,speed*PI/25,&D_target,&Q_target);//��ʱvoltage_amp�ĵ�λ��0.01v			
//								motor_run_DQ_new(Q_target,10.0/voltage_source*(voltage_amp/100.0),phase);//��Ȧ�ڵ����ӳٲ���
//								delay_ms(10);
//								
//								current_1=get_current1;  //32%
//								current_2=get_current2;
//								current_3=get_current3;
//								current_4=get_current4;								
//								Four_phase_clark(current_1,current_2,current_3,current_4,&ialphi,&ibeta);  //7%
//								Park(ialphi,ibeta,phase/100.0+angel,&id,&iq);
//								
//								PID_value_Q=0;
								
								
                USART_RX_STA=0;
                return;
            }
            if(memcmp(cmd_split,"ang",sizeof(cmd_split)/sizeof(cmd_split[0]))==0) {
                cmd_split = strtok(NULL," ");
                recive_num=0;
                for(t=0; t<get_length(cmd_split); t++)
                {
                    recive_num=cmd_split[t]-'0'+recive_num*10;
                }
                angel=recive_num*0.01;
                printf("change ang %lf\n",angel);
                USART_RX_STA=0;
                return;
            }
            if(memcmp(cmd_split,"dif",sizeof(cmd_split)/sizeof(cmd_split[0]))==0) {
                cmd_split = strtok(NULL," ");
                recive_num=0;
                for(t=0; t<get_length(cmd_split); t++)
                {
                    recive_num=cmd_split[t]-'0'+recive_num*10;
                }
                phase_dif=recive_num*0.01;
                printf("change phase_dif %lf\n",phase_dif);

                USART_RX_STA=0;
                return;
            }
            if(memcmp(cmd_split,"pha",sizeof(cmd_split)/sizeof(cmd_split[0]))==0) {
                cmd_split = strtok(NULL," ");
                recive_num=0;
                for(t=0; t<get_length(cmd_split); t++)
                {
                    recive_num=cmd_split[t]-'0'+recive_num*10;
                }
                phase=recive_num;
                printf("change phase %f\n",phase*0.01);

                USART_RX_STA=0;
                return;
            }
            if(memcmp(cmd_split,"start",sizeof(cmd_split)/sizeof(cmd_split[0]))==0) {

                printf("ok\n");

                USART_RX_STA=0;
                return;
            }
            if(memcmp(cmd_split,"gethal",sizeof(cmd_split)/sizeof(cmd_split[0]))==0) {

                get_hall=get_hall*(-1);

                printf("hall is %d\n",get_hall);

                USART_RX_STA=0;
                return;
            }
            if(memcmp(cmd_split,"getcur",sizeof(cmd_split)/sizeof(cmd_split[0]))==0) {

                get_cur=get_cur*(-1);

                printf("curent_state is %d An",get_cur);

                USART_RX_STA=0;
                return;
            }
            if(memcmp(cmd_split,"del",sizeof(cmd_split)/sizeof(cmd_split[0]))==0) {
                cmd_split = strtok(NULL," ");
                recive_num=0;
                for(t=0; t<get_length(cmd_split); t++)
                {
                    recive_num=cmd_split[t]-'0'+recive_num*10;
                }
                printf("change delaytime %d\n",recive_num);
                delaytime=recive_num;
                USART_RX_STA=0;
                return;
            }
            if(memcmp(cmd_split,"ckp",sizeof(cmd_split)/sizeof(cmd_split[0]))==0) {
                cmd_split = strtok(NULL," ");
                recive_num=0;
                kp_current=strtod(cmd_split,NULL);
                printf("change kp_current %lf\n",kp_current);;
                USART_RX_STA=0;
                return;
            }
            if(memcmp(cmd_split,"cki",sizeof(cmd_split)/sizeof(cmd_split[0]))==0) {
                cmd_split = strtok(NULL," ");
                recive_num=0;
                ki_current=strtod(cmd_split,NULL);
                printf("change ki_current %lf\n",ki_current);
                USART_RX_STA=0;
                return;
            }
            if(memcmp(cmd_split,"ckd",sizeof(cmd_split)/sizeof(cmd_split[0]))==0) {
                cmd_split = strtok(NULL," ");
                recive_num=0;
                kd_current=strtod(cmd_split,NULL);
                printf("change kd_current %lf\n",kd_current);
                USART_RX_STA=0;
                return;
            }
						if(memcmp(cmd_split,"net",sizeof(cmd_split)/sizeof(cmd_split[0]))==0) {
                cmd_split = strtok(NULL," ");
                recive_num=0;
                
								//  �޸�mpc����						
								R_add=strtod(cmd_split,NULL);
								cmd_split = strtok(NULL," ");
								L_add=strtod(cmd_split,NULL);
								cmd_split = strtok(NULL," ");
								Ke_add=strtod(cmd_split,NULL);
							
                printf("ok\n");
							//printf("R:%lf,L:%lf,Ke:%lf\n",);
                USART_RX_STA=0;
                return;
            }
            delay_ms(10);
            USART_RX_STA=0;
        }
    }
}


//��ȡ�ַ�������
int get_length(char str[])
{
    char *p = str;
    int count = 0;
    while (*p++ != '\0')
    {
        count++;
    }
    return count;
}

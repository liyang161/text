#include "park.h"
#include <math.h>
#include "IQmathLib.h"
//四相电流的clark变换
void clark_and_park(double i1,double i2,double i3,double i4,double angel,double *d,double *q){
	double pi=3.14159;
	//_iq a=(_IQ(pi));
	_iq a=(_IQ(1.2345));
	float b=_IQtoF(a);
  //*ialphi=i1*sin(0)+i2*sin(3.0/4*pi)+i3*sin(3.0/4*pi*2)+i4*sin(3.0/4*pi*3);
  //*ibeta=i1*cos(0)+i2*cos(3.0/4*pi)+i3*cos(3.0/4*pi*2)+i4*cos(3.0/4*pi*3);
	*d=i1*sin(0+angel)+i2*sin(3.0/4*pi+angel)+i3*sin(3.0/4*pi*2+angel)+i4*sin(3.0/4*pi*3+angel);
  *q=i1*cos(0+angel)+i2*cos(3.0/4*pi+angel)+i3*cos(3.0/4*pi*2+angel)+i4*cos(3.0/4*pi*3+angel);

}
//四相电流的clark变换
void Four_phase_clark(double i1,double i2,double i3,double i4,double *ialphi,double *ibeta){
	double pi=3.1415926;
	double angel=0;//可能出错 
  //*ialphi=i1*sin(0)+i2*sin(3.0/4*pi)+i3*sin(3.0/4*pi*2)+i4*sin(3.0/4*pi*3);
  //*ibeta=i1*cos(0)+i2*cos(3.0/4*pi)+i3*cos(3.0/4*pi*2)+i4*cos(3.0/4*pi*3);
	*ialphi=i1*sin(0+angel)+i2*sin(3.0/4*pi+angel)+i3*sin(3.0/4*pi*2+angel)+i4*sin(3.0/4*pi*3+angel);
  *ibeta=i1*cos(0+angel)+i2*cos(3.0/4*pi+angel)+i3*cos(3.0/4*pi*2+angel)+i4*cos(3.0/4*pi*3+angel);


}
//park变换
void Park(double alpha,double beta,double theta,double *d,double *q){
	//theta+=0.6; //加上初始相位，否则出错
	
	//*d=alpha*cos(theta)+beta*sin(theta);//D轴对齐
	//*q=-alpha*sin(theta)+beta*cos(theta);
	
	
	*d=alpha*sin(theta)-beta*cos(theta);//Q轴对齐
	*q=alpha*cos(theta)+beta*sin(theta);
	
}
//逆park变换
void Park_inverse(double d,double q,double theta,double *alpha,double *beta){
	
	//theta+=0.6;
	
	//*alpha=d*cos(theta)-q*sin(theta);//D轴对齐
	//*beta=d*sin(theta)+q*cos(theta);
	
	*alpha=d*sin(theta)+q*cos(theta);//Q轴对齐
	*beta=-d*cos(theta)+q*sin(theta);
	
}
//四相电流的逆clark变换
void Four_phase_clark_inverse(double ialphi,double ibeta,double *i1,double *i2,double *i3,double *i4){
	double pi=3.1415926;
  *i1=ialphi*sin(0)+ibeta*cos(0);
  *i2=ialphi*sin(3.0/4*pi)+ibeta*cos(3.0/4*pi);
  *i3=ialphi*sin(3.0/4*pi*2)+ibeta*cos(3.0/4*pi*2);
  *i4=ialphi*sin(3.0/4*pi*3)+ibeta*cos(3.0/4*pi*3);
}
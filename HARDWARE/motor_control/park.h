#ifndef __PARK_H
#define __PARK_H

void Four_phase_clark(double i1,double i2,double i3,double i4,double *ialphi,double *ibeta);
void Park(double alpha,double beta,double theta,double *d,double *q);
void Park_inverse(double d,double q,double theta,double *alpha,double *beta);
void Four_phase_clark_inverse(double ialphi,double ibeta,double *i1,double *i2,double *i3,double *i4);
void clark_and_park(double i1,double i2,double i3,double i4,double angel,double *d,double *q);
#endif
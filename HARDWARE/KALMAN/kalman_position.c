/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: kalman_position.c
 *
 * Code generated for Simulink model 'kalman_position'.
 *
 * Model version                  : 1.57
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Mon Jun 19 10:32:11 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "kalman_position.h"
#include "rtwtypes.h"
#include <math.h>
#include "rt_nonfinite.h"
#include <string.h>
#include "kalman_position_private.h"

/* Block signals (default storage) */
B_kalman_position_T kalman_position_B;

/* Block states (default storage) */
DW_kalman_position_T kalman_position_DW;

/* External inputs (root inport signals with default storage) */
ExtU_kalman_position_T kalman_position_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_kalman_position_T kalman_position_Y;

/* Real-time model */
static RT_MODEL_kalman_position_T kalman_position_M_;
RT_MODEL_kalman_position_T *const kalman_position_M = &kalman_position_M_;

/* Forward declaration for local functions */
static real_T kalman_position_xnrm2(int32_T n, const real_T x[12], int32_T ix0);
static void kalman_position_xgemv(int32_T m, int32_T n, const real_T A[12],
  int32_T ia0, const real_T x[12], int32_T ix0, real_T y[2]);
static void kalman_position_xgerc(int32_T m, int32_T n, real_T alpha1, int32_T
  ix0, const real_T y[2], real_T A[12], int32_T ia0);
static void kalman_position_qrFactor(const real_T A[8], const real_T S[16],
  const real_T Ns[4], real_T b_S[4]);
static void kalman_position_trisolve(const real_T A[4], real_T B[8]);
static void kalman_position_trisolve_c(const real_T A[4], real_T B[8]);
static real_T kalman_position_xnrm2_h(int32_T n, const real_T x[24], int32_T ix0);
static void kalman_position_xgemv_b(int32_T m, int32_T n, const real_T A[24],
  int32_T ia0, const real_T x[24], int32_T ix0, real_T y[4]);
static void kalman_position_xgerc_l(int32_T m, int32_T n, real_T alpha1, int32_T
  ix0, const real_T y[4], real_T A[24], int32_T ia0);
static void kalman_position_qrFactor_j(const real_T A[16], real_T S[16], const
  real_T Ns[8]);
static real_T kalman_position_xnrm2_d(int32_T n, const real_T x[32], int32_T ix0);
static void kalman_position_xgemv_l(int32_T m, int32_T n, const real_T A[32],
  int32_T ia0, const real_T x[32], int32_T ix0, real_T y[4]);
static void kalman_position_xgerc_i(int32_T m, int32_T n, real_T alpha1, int32_T
  ix0, const real_T y[4], real_T A[32], int32_T ia0);
static void kalman_position_qrFactor_k(const real_T A[16], real_T S[16], const
  real_T Ns[16]);

/* Function for MATLAB Function: '<S2>/Correct' */
static real_T kalman_position_xnrm2(int32_T n, const real_T x[12], int32_T ix0)
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  int32_T k;
  int32_T kend;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T y;
  a = fabs(u0);
  y = fabs(u1);
  if (a < y) {
    a /= y;
    y *= sqrt(a * a + 1.0);
  } else if (a > y) {
    y /= a;
    y = sqrt(y * y + 1.0) * a;
  } else if (!rtIsNaN(y)) {
    y = a * 1.4142135623730951;
  }

  return y;
}

/* Function for MATLAB Function: '<S2>/Correct' */
static void kalman_position_xgemv(int32_T m, int32_T n, const real_T A[12],
  int32_T ia0, const real_T x[12], int32_T ix0, real_T y[2])
{
  real_T c;
  int32_T b;
  int32_T b_iy;
  int32_T d;
  int32_T ia;
  int32_T iac;
  int32_T ix;
  if ((m != 0) && (n != 0)) {
    for (b_iy = 0; b_iy < n; b_iy++) {
      y[b_iy] = 0.0;
    }

    b_iy = 0;
    b = (n - 1) * 6 + ia0;
    for (iac = ia0; iac <= b; iac += 6) {
      ix = ix0;
      c = 0.0;
      d = (iac + m) - 1;
      for (ia = iac; ia <= d; ia++) {
        c += A[ia - 1] * x[ix - 1];
        ix++;
      }

      y[b_iy] += c;
      b_iy++;
    }
  }
}

/* Function for MATLAB Function: '<S2>/Correct' */
static void kalman_position_xgerc(int32_T m, int32_T n, real_T alpha1, int32_T
  ix0, const real_T y[2], real_T A[12], int32_T ia0)
{
  real_T temp;
  int32_T b;
  int32_T ijA;
  int32_T ix;
  int32_T j;
  int32_T jA;
  int32_T jy;
  if (!(alpha1 == 0.0)) {
    jA = ia0 - 1;
    jy = 0;
    for (j = 0; j < n; j++) {
      if (y[jy] != 0.0) {
        temp = y[jy] * alpha1;
        ix = ix0;
        ijA = jA;
        b = m + jA;
        while (ijA + 1 <= b) {
          A[ijA] += A[ix - 1] * temp;
          ix++;
          ijA++;
        }
      }

      jy++;
      jA += 6;
    }
  }
}

/* Function for MATLAB Function: '<S2>/Correct' */
static void kalman_position_qrFactor(const real_T A[8], const real_T S[16],
  const real_T Ns[4], real_T b_S[4])
{
  int32_T aoffset;
  int32_T coffset;
  int32_T exitg1;
  int32_T knt;
  int32_T lastc;
  for (lastc = 0; lastc < 2; lastc++) {
    coffset = lastc << 2;
    for (knt = 0; knt < 4; knt++) {
      aoffset = knt << 2;
      kalman_position_B.y_cx[coffset + knt] = ((S[aoffset + 1] * A[lastc + 2] +
        S[aoffset] * A[lastc]) + S[aoffset + 2] * A[lastc + 4]) + S[aoffset + 3]
        * A[lastc + 6];
    }
  }

  for (knt = 0; knt < 2; knt++) {
    lastc = knt << 2;
    kalman_position_B.b_A_k[6 * knt] = kalman_position_B.y_cx[lastc];
    kalman_position_B.b_A_k[6 * knt + 1] = kalman_position_B.y_cx[lastc + 1];
    kalman_position_B.b_A_k[6 * knt + 2] = kalman_position_B.y_cx[lastc + 2];
    kalman_position_B.b_A_k[6 * knt + 3] = kalman_position_B.y_cx[lastc + 3];
    kalman_position_B.b_A_k[6 * knt + 4] = Ns[knt];
    kalman_position_B.b_A_k[6 * knt + 5] = Ns[knt + 2];
    kalman_position_B.work_c[knt] = 0.0;
  }

  kalman_position_B.atmp_m = kalman_position_B.b_A_k[0];
  kalman_position_B.tau_idx_0_p = 0.0;
  kalman_position_B.beta1_n = kalman_position_xnrm2(5, kalman_position_B.b_A_k,
    2);
  if (kalman_position_B.beta1_n != 0.0) {
    kalman_position_B.beta1_n = rt_hypotd_snf(kalman_position_B.b_A_k[0],
      kalman_position_B.beta1_n);
    if (kalman_position_B.b_A_k[0] >= 0.0) {
      kalman_position_B.beta1_n = -kalman_position_B.beta1_n;
    }

    if (fabs(kalman_position_B.beta1_n) < 1.0020841800044864E-292) {
      knt = 0;
      do {
        knt++;
        for (lastc = 1; lastc < 6; lastc++) {
          kalman_position_B.b_A_k[lastc] *= 9.9792015476736E+291;
        }

        kalman_position_B.beta1_n *= 9.9792015476736E+291;
        kalman_position_B.atmp_m *= 9.9792015476736E+291;
      } while ((fabs(kalman_position_B.beta1_n) < 1.0020841800044864E-292) &&
               (knt < 20));

      kalman_position_B.beta1_n = rt_hypotd_snf(kalman_position_B.atmp_m,
        kalman_position_xnrm2(5, kalman_position_B.b_A_k, 2));
      if (kalman_position_B.atmp_m >= 0.0) {
        kalman_position_B.beta1_n = -kalman_position_B.beta1_n;
      }

      kalman_position_B.tau_idx_0_p = (kalman_position_B.beta1_n -
        kalman_position_B.atmp_m) / kalman_position_B.beta1_n;
      kalman_position_B.atmp_m = 1.0 / (kalman_position_B.atmp_m -
        kalman_position_B.beta1_n);
      for (lastc = 1; lastc < 6; lastc++) {
        kalman_position_B.b_A_k[lastc] *= kalman_position_B.atmp_m;
      }

      for (lastc = 0; lastc < knt; lastc++) {
        kalman_position_B.beta1_n *= 1.0020841800044864E-292;
      }

      kalman_position_B.atmp_m = kalman_position_B.beta1_n;
    } else {
      kalman_position_B.tau_idx_0_p = (kalman_position_B.beta1_n -
        kalman_position_B.b_A_k[0]) / kalman_position_B.beta1_n;
      kalman_position_B.atmp_m = 1.0 / (kalman_position_B.b_A_k[0] -
        kalman_position_B.beta1_n);
      for (knt = 1; knt < 6; knt++) {
        kalman_position_B.b_A_k[knt] *= kalman_position_B.atmp_m;
      }

      kalman_position_B.atmp_m = kalman_position_B.beta1_n;
    }
  }

  kalman_position_B.b_A_k[0] = 1.0;
  if (kalman_position_B.tau_idx_0_p != 0.0) {
    knt = 6;
    lastc = 5;
    while ((knt > 0) && (kalman_position_B.b_A_k[lastc] == 0.0)) {
      knt--;
      lastc--;
    }

    lastc = 1;
    coffset = 0;
    do {
      exitg1 = 0;
      if (coffset + 7 <= knt + 6) {
        if (kalman_position_B.b_A_k[coffset + 6] != 0.0) {
          exitg1 = 1;
        } else {
          coffset++;
        }
      } else {
        lastc = 0;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    knt = 0;
    lastc = 0;
  }

  if (knt > 0) {
    kalman_position_xgemv(knt, lastc, kalman_position_B.b_A_k, 7,
                          kalman_position_B.b_A_k, 1, kalman_position_B.work_c);
    kalman_position_xgerc(knt, lastc, -kalman_position_B.tau_idx_0_p, 1,
                          kalman_position_B.work_c, kalman_position_B.b_A_k, 7);
  }

  kalman_position_B.b_A_k[0] = kalman_position_B.atmp_m;
  kalman_position_B.atmp_m = kalman_position_B.b_A_k[7];
  kalman_position_B.beta1_n = kalman_position_xnrm2(4, kalman_position_B.b_A_k,
    9);
  if (kalman_position_B.beta1_n != 0.0) {
    kalman_position_B.beta1_n = rt_hypotd_snf(kalman_position_B.b_A_k[7],
      kalman_position_B.beta1_n);
    if (kalman_position_B.b_A_k[7] >= 0.0) {
      kalman_position_B.beta1_n = -kalman_position_B.beta1_n;
    }

    if (fabs(kalman_position_B.beta1_n) < 1.0020841800044864E-292) {
      knt = 0;
      do {
        knt++;
        for (lastc = 8; lastc < 12; lastc++) {
          kalman_position_B.b_A_k[lastc] *= 9.9792015476736E+291;
        }

        kalman_position_B.beta1_n *= 9.9792015476736E+291;
        kalman_position_B.atmp_m *= 9.9792015476736E+291;
      } while ((fabs(kalman_position_B.beta1_n) < 1.0020841800044864E-292) &&
               (knt < 20));

      kalman_position_B.beta1_n = rt_hypotd_snf(kalman_position_B.atmp_m,
        kalman_position_xnrm2(4, kalman_position_B.b_A_k, 9));
      if (kalman_position_B.atmp_m >= 0.0) {
        kalman_position_B.beta1_n = -kalman_position_B.beta1_n;
      }

      kalman_position_B.atmp_m = 1.0 / (kalman_position_B.atmp_m -
        kalman_position_B.beta1_n);
      for (lastc = 8; lastc < 12; lastc++) {
        kalman_position_B.b_A_k[lastc] *= kalman_position_B.atmp_m;
      }

      for (lastc = 0; lastc < knt; lastc++) {
        kalman_position_B.beta1_n *= 1.0020841800044864E-292;
      }

      kalman_position_B.atmp_m = kalman_position_B.beta1_n;
    } else {
      kalman_position_B.atmp_m = 1.0 / (kalman_position_B.b_A_k[7] -
        kalman_position_B.beta1_n);
      for (knt = 8; knt < 12; knt++) {
        kalman_position_B.b_A_k[knt] *= kalman_position_B.atmp_m;
      }

      kalman_position_B.atmp_m = kalman_position_B.beta1_n;
    }
  }

  kalman_position_B.b_A_k[7] = kalman_position_B.atmp_m;
  kalman_position_B.R[0] = kalman_position_B.b_A_k[0];
  for (knt = 0; knt < 2; knt++) {
    kalman_position_B.R[knt + 2] = kalman_position_B.b_A_k[knt + 6];
  }

  b_S[0] = kalman_position_B.R[0];
  b_S[1] = kalman_position_B.R[2];
  b_S[2] = 0.0;
  b_S[3] = kalman_position_B.R[3];
}

/* Function for MATLAB Function: '<S2>/Correct' */
static void kalman_position_trisolve(const real_T A[4], real_T B[8])
{
  int32_T j;
  int32_T jBcol;
  for (j = 0; j < 4; j++) {
    jBcol = j << 1;
    if (B[jBcol] != 0.0) {
      B[jBcol] /= A[0];
      B[jBcol + 1] -= B[jBcol] * A[1];
    }

    if (B[jBcol + 1] != 0.0) {
      B[jBcol + 1] /= A[3];
    }
  }
}

/* Function for MATLAB Function: '<S2>/Correct' */
static void kalman_position_trisolve_c(const real_T A[4], real_T B[8])
{
  real_T tmp;
  int32_T j;
  int32_T jBcol;
  for (j = 0; j < 4; j++) {
    jBcol = j << 1;
    tmp = B[jBcol + 1];
    if (tmp != 0.0) {
      B[jBcol + 1] = tmp / A[3];
      B[jBcol] -= B[jBcol + 1] * A[2];
    }

    if (B[jBcol] != 0.0) {
      B[jBcol] /= A[0];
    }
  }
}

/* Function for MATLAB Function: '<S2>/Correct' */
static real_T kalman_position_xnrm2_h(int32_T n, const real_T x[24], int32_T ix0)
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  int32_T k;
  int32_T kend;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S2>/Correct' */
static void kalman_position_xgemv_b(int32_T m, int32_T n, const real_T A[24],
  int32_T ia0, const real_T x[24], int32_T ix0, real_T y[4])
{
  real_T c;
  int32_T b;
  int32_T b_iy;
  int32_T d;
  int32_T ia;
  int32_T iac;
  int32_T ix;
  if ((m != 0) && (n != 0)) {
    for (b_iy = 0; b_iy < n; b_iy++) {
      y[b_iy] = 0.0;
    }

    b_iy = 0;
    b = (n - 1) * 6 + ia0;
    for (iac = ia0; iac <= b; iac += 6) {
      ix = ix0;
      c = 0.0;
      d = (iac + m) - 1;
      for (ia = iac; ia <= d; ia++) {
        c += A[ia - 1] * x[ix - 1];
        ix++;
      }

      y[b_iy] += c;
      b_iy++;
    }
  }
}

/* Function for MATLAB Function: '<S2>/Correct' */
static void kalman_position_xgerc_l(int32_T m, int32_T n, real_T alpha1, int32_T
  ix0, const real_T y[4], real_T A[24], int32_T ia0)
{
  real_T temp;
  int32_T b;
  int32_T ijA;
  int32_T ix;
  int32_T j;
  int32_T jA;
  int32_T jy;
  if (!(alpha1 == 0.0)) {
    jA = ia0 - 1;
    jy = 0;
    for (j = 0; j < n; j++) {
      if (y[jy] != 0.0) {
        temp = y[jy] * alpha1;
        ix = ix0;
        ijA = jA;
        b = m + jA;
        while (ijA + 1 <= b) {
          A[ijA] += A[ix - 1] * temp;
          ix++;
          ijA++;
        }
      }

      jy++;
      jA += 6;
    }
  }
}

/* Function for MATLAB Function: '<S2>/Correct' */
static void kalman_position_qrFactor_j(const real_T A[16], real_T S[16], const
  real_T Ns[8])
{
  int32_T aoffset;
  int32_T coffset;
  int32_T exitg1;
  int32_T knt;
  int32_T lastc;
  boolean_T exitg2;
  for (lastc = 0; lastc < 4; lastc++) {
    coffset = lastc << 2;
    for (knt = 0; knt < 4; knt++) {
      aoffset = knt << 2;
      kalman_position_B.y_c[coffset + knt] = ((S[aoffset + 1] * A[lastc + 4] +
        S[aoffset] * A[lastc]) + S[aoffset + 2] * A[lastc + 8]) + S[aoffset + 3]
        * A[lastc + 12];
    }
  }

  for (knt = 0; knt < 4; knt++) {
    lastc = knt << 2;
    kalman_position_B.b_A_m[6 * knt] = kalman_position_B.y_c[lastc];
    kalman_position_B.b_A_m[6 * knt + 1] = kalman_position_B.y_c[lastc + 1];
    kalman_position_B.b_A_m[6 * knt + 2] = kalman_position_B.y_c[lastc + 2];
    kalman_position_B.b_A_m[6 * knt + 3] = kalman_position_B.y_c[lastc + 3];
    kalman_position_B.b_A_m[6 * knt + 4] = Ns[knt];
    kalman_position_B.b_A_m[6 * knt + 5] = Ns[knt + 4];
    kalman_position_B.work_p[knt] = 0.0;
  }

  kalman_position_B.atmp_f = kalman_position_B.b_A_m[0];
  kalman_position_B.tau_idx_0_g = 0.0;
  kalman_position_B.beta1_g = kalman_position_xnrm2_h(5, kalman_position_B.b_A_m,
    2);
  if (kalman_position_B.beta1_g != 0.0) {
    kalman_position_B.beta1_g = rt_hypotd_snf(kalman_position_B.b_A_m[0],
      kalman_position_B.beta1_g);
    if (kalman_position_B.b_A_m[0] >= 0.0) {
      kalman_position_B.beta1_g = -kalman_position_B.beta1_g;
    }

    if (fabs(kalman_position_B.beta1_g) < 1.0020841800044864E-292) {
      knt = 0;
      do {
        knt++;
        for (lastc = 1; lastc < 6; lastc++) {
          kalman_position_B.b_A_m[lastc] *= 9.9792015476736E+291;
        }

        kalman_position_B.beta1_g *= 9.9792015476736E+291;
        kalman_position_B.atmp_f *= 9.9792015476736E+291;
      } while ((fabs(kalman_position_B.beta1_g) < 1.0020841800044864E-292) &&
               (knt < 20));

      kalman_position_B.beta1_g = rt_hypotd_snf(kalman_position_B.atmp_f,
        kalman_position_xnrm2_h(5, kalman_position_B.b_A_m, 2));
      if (kalman_position_B.atmp_f >= 0.0) {
        kalman_position_B.beta1_g = -kalman_position_B.beta1_g;
      }

      kalman_position_B.tau_idx_0_g = (kalman_position_B.beta1_g -
        kalman_position_B.atmp_f) / kalman_position_B.beta1_g;
      kalman_position_B.atmp_f = 1.0 / (kalman_position_B.atmp_f -
        kalman_position_B.beta1_g);
      for (lastc = 1; lastc < 6; lastc++) {
        kalman_position_B.b_A_m[lastc] *= kalman_position_B.atmp_f;
      }

      for (lastc = 0; lastc < knt; lastc++) {
        kalman_position_B.beta1_g *= 1.0020841800044864E-292;
      }

      kalman_position_B.atmp_f = kalman_position_B.beta1_g;
    } else {
      kalman_position_B.tau_idx_0_g = (kalman_position_B.beta1_g -
        kalman_position_B.b_A_m[0]) / kalman_position_B.beta1_g;
      kalman_position_B.atmp_f = 1.0 / (kalman_position_B.b_A_m[0] -
        kalman_position_B.beta1_g);
      for (knt = 1; knt < 6; knt++) {
        kalman_position_B.b_A_m[knt] *= kalman_position_B.atmp_f;
      }

      kalman_position_B.atmp_f = kalman_position_B.beta1_g;
    }
  }

  kalman_position_B.b_A_m[0] = 1.0;
  if (kalman_position_B.tau_idx_0_g != 0.0) {
    knt = 6;
    lastc = 5;
    while ((knt > 0) && (kalman_position_B.b_A_m[lastc] == 0.0)) {
      knt--;
      lastc--;
    }

    lastc = 3;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      coffset = (lastc - 1) * 6 + 6;
      aoffset = coffset;
      do {
        exitg1 = 0;
        if (aoffset + 1 <= coffset + knt) {
          if (kalman_position_B.b_A_m[aoffset] != 0.0) {
            exitg1 = 1;
          } else {
            aoffset++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    knt = 0;
    lastc = 0;
  }

  if (knt > 0) {
    kalman_position_xgemv_b(knt, lastc, kalman_position_B.b_A_m, 7,
      kalman_position_B.b_A_m, 1, kalman_position_B.work_p);
    kalman_position_xgerc_l(knt, lastc, -kalman_position_B.tau_idx_0_g, 1,
      kalman_position_B.work_p, kalman_position_B.b_A_m, 7);
  }

  kalman_position_B.b_A_m[0] = kalman_position_B.atmp_f;
  kalman_position_B.atmp_f = kalman_position_B.b_A_m[7];
  kalman_position_B.tau_idx_0_g = 0.0;
  kalman_position_B.beta1_g = kalman_position_xnrm2_h(4, kalman_position_B.b_A_m,
    9);
  if (kalman_position_B.beta1_g != 0.0) {
    kalman_position_B.beta1_g = rt_hypotd_snf(kalman_position_B.b_A_m[7],
      kalman_position_B.beta1_g);
    if (kalman_position_B.b_A_m[7] >= 0.0) {
      kalman_position_B.beta1_g = -kalman_position_B.beta1_g;
    }

    if (fabs(kalman_position_B.beta1_g) < 1.0020841800044864E-292) {
      knt = 0;
      do {
        knt++;
        for (lastc = 8; lastc < 12; lastc++) {
          kalman_position_B.b_A_m[lastc] *= 9.9792015476736E+291;
        }

        kalman_position_B.beta1_g *= 9.9792015476736E+291;
        kalman_position_B.atmp_f *= 9.9792015476736E+291;
      } while ((fabs(kalman_position_B.beta1_g) < 1.0020841800044864E-292) &&
               (knt < 20));

      kalman_position_B.beta1_g = rt_hypotd_snf(kalman_position_B.atmp_f,
        kalman_position_xnrm2_h(4, kalman_position_B.b_A_m, 9));
      if (kalman_position_B.atmp_f >= 0.0) {
        kalman_position_B.beta1_g = -kalman_position_B.beta1_g;
      }

      kalman_position_B.tau_idx_0_g = (kalman_position_B.beta1_g -
        kalman_position_B.atmp_f) / kalman_position_B.beta1_g;
      kalman_position_B.atmp_f = 1.0 / (kalman_position_B.atmp_f -
        kalman_position_B.beta1_g);
      for (lastc = 8; lastc < 12; lastc++) {
        kalman_position_B.b_A_m[lastc] *= kalman_position_B.atmp_f;
      }

      for (lastc = 0; lastc < knt; lastc++) {
        kalman_position_B.beta1_g *= 1.0020841800044864E-292;
      }

      kalman_position_B.atmp_f = kalman_position_B.beta1_g;
    } else {
      kalman_position_B.tau_idx_0_g = (kalman_position_B.beta1_g -
        kalman_position_B.b_A_m[7]) / kalman_position_B.beta1_g;
      kalman_position_B.atmp_f = 1.0 / (kalman_position_B.b_A_m[7] -
        kalman_position_B.beta1_g);
      for (knt = 8; knt < 12; knt++) {
        kalman_position_B.b_A_m[knt] *= kalman_position_B.atmp_f;
      }

      kalman_position_B.atmp_f = kalman_position_B.beta1_g;
    }
  }

  kalman_position_B.b_A_m[7] = 1.0;
  if (kalman_position_B.tau_idx_0_g != 0.0) {
    knt = 5;
    lastc = 11;
    while ((knt > 0) && (kalman_position_B.b_A_m[lastc] == 0.0)) {
      knt--;
      lastc--;
    }

    lastc = 2;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      coffset = (lastc - 1) * 6 + 13;
      aoffset = coffset;
      do {
        exitg1 = 0;
        if (aoffset + 1 <= coffset + knt) {
          if (kalman_position_B.b_A_m[aoffset] != 0.0) {
            exitg1 = 1;
          } else {
            aoffset++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    knt = 0;
    lastc = 0;
  }

  if (knt > 0) {
    kalman_position_xgemv_b(knt, lastc, kalman_position_B.b_A_m, 14,
      kalman_position_B.b_A_m, 8, kalman_position_B.work_p);
    kalman_position_xgerc_l(knt, lastc, -kalman_position_B.tau_idx_0_g, 8,
      kalman_position_B.work_p, kalman_position_B.b_A_m, 14);
  }

  kalman_position_B.b_A_m[7] = kalman_position_B.atmp_f;
  kalman_position_B.atmp_f = kalman_position_B.b_A_m[14];
  kalman_position_B.tau_idx_0_g = 0.0;
  kalman_position_B.beta1_g = kalman_position_xnrm2_h(3, kalman_position_B.b_A_m,
    16);
  if (kalman_position_B.beta1_g != 0.0) {
    kalman_position_B.beta1_g = rt_hypotd_snf(kalman_position_B.b_A_m[14],
      kalman_position_B.beta1_g);
    if (kalman_position_B.b_A_m[14] >= 0.0) {
      kalman_position_B.beta1_g = -kalman_position_B.beta1_g;
    }

    if (fabs(kalman_position_B.beta1_g) < 1.0020841800044864E-292) {
      knt = 0;
      do {
        knt++;
        for (lastc = 15; lastc < 18; lastc++) {
          kalman_position_B.b_A_m[lastc] *= 9.9792015476736E+291;
        }

        kalman_position_B.beta1_g *= 9.9792015476736E+291;
        kalman_position_B.atmp_f *= 9.9792015476736E+291;
      } while ((fabs(kalman_position_B.beta1_g) < 1.0020841800044864E-292) &&
               (knt < 20));

      kalman_position_B.beta1_g = rt_hypotd_snf(kalman_position_B.atmp_f,
        kalman_position_xnrm2_h(3, kalman_position_B.b_A_m, 16));
      if (kalman_position_B.atmp_f >= 0.0) {
        kalman_position_B.beta1_g = -kalman_position_B.beta1_g;
      }

      kalman_position_B.tau_idx_0_g = (kalman_position_B.beta1_g -
        kalman_position_B.atmp_f) / kalman_position_B.beta1_g;
      kalman_position_B.atmp_f = 1.0 / (kalman_position_B.atmp_f -
        kalman_position_B.beta1_g);
      for (lastc = 15; lastc < 18; lastc++) {
        kalman_position_B.b_A_m[lastc] *= kalman_position_B.atmp_f;
      }

      for (lastc = 0; lastc < knt; lastc++) {
        kalman_position_B.beta1_g *= 1.0020841800044864E-292;
      }

      kalman_position_B.atmp_f = kalman_position_B.beta1_g;
    } else {
      kalman_position_B.tau_idx_0_g = (kalman_position_B.beta1_g -
        kalman_position_B.b_A_m[14]) / kalman_position_B.beta1_g;
      kalman_position_B.atmp_f = 1.0 / (kalman_position_B.b_A_m[14] -
        kalman_position_B.beta1_g);
      for (knt = 15; knt < 18; knt++) {
        kalman_position_B.b_A_m[knt] *= kalman_position_B.atmp_f;
      }

      kalman_position_B.atmp_f = kalman_position_B.beta1_g;
    }
  }

  kalman_position_B.b_A_m[14] = 1.0;
  if (kalman_position_B.tau_idx_0_g != 0.0) {
    knt = 4;
    lastc = 17;
    while ((knt > 0) && (kalman_position_B.b_A_m[lastc] == 0.0)) {
      knt--;
      lastc--;
    }

    lastc = 1;
    aoffset = 20;
    do {
      exitg1 = 0;
      if (aoffset + 1 <= knt + 20) {
        if (kalman_position_B.b_A_m[aoffset] != 0.0) {
          exitg1 = 1;
        } else {
          aoffset++;
        }
      } else {
        lastc = 0;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    knt = 0;
    lastc = 0;
  }

  if (knt > 0) {
    kalman_position_xgemv_b(knt, lastc, kalman_position_B.b_A_m, 21,
      kalman_position_B.b_A_m, 15, kalman_position_B.work_p);
    kalman_position_xgerc_l(knt, lastc, -kalman_position_B.tau_idx_0_g, 15,
      kalman_position_B.work_p, kalman_position_B.b_A_m, 21);
  }

  kalman_position_B.b_A_m[14] = kalman_position_B.atmp_f;
  kalman_position_B.atmp_f = kalman_position_B.b_A_m[21];
  kalman_position_B.beta1_g = kalman_position_xnrm2_h(2, kalman_position_B.b_A_m,
    23);
  if (kalman_position_B.beta1_g != 0.0) {
    kalman_position_B.beta1_g = rt_hypotd_snf(kalman_position_B.b_A_m[21],
      kalman_position_B.beta1_g);
    if (kalman_position_B.b_A_m[21] >= 0.0) {
      kalman_position_B.beta1_g = -kalman_position_B.beta1_g;
    }

    if (fabs(kalman_position_B.beta1_g) < 1.0020841800044864E-292) {
      knt = 0;
      do {
        knt++;
        for (lastc = 22; lastc < 24; lastc++) {
          kalman_position_B.b_A_m[lastc] *= 9.9792015476736E+291;
        }

        kalman_position_B.beta1_g *= 9.9792015476736E+291;
        kalman_position_B.atmp_f *= 9.9792015476736E+291;
      } while ((fabs(kalman_position_B.beta1_g) < 1.0020841800044864E-292) &&
               (knt < 20));

      kalman_position_B.beta1_g = rt_hypotd_snf(kalman_position_B.atmp_f,
        kalman_position_xnrm2_h(2, kalman_position_B.b_A_m, 23));
      if (kalman_position_B.atmp_f >= 0.0) {
        kalman_position_B.beta1_g = -kalman_position_B.beta1_g;
      }

      kalman_position_B.atmp_f = 1.0 / (kalman_position_B.atmp_f -
        kalman_position_B.beta1_g);
      for (lastc = 22; lastc < 24; lastc++) {
        kalman_position_B.b_A_m[lastc] *= kalman_position_B.atmp_f;
      }

      for (lastc = 0; lastc < knt; lastc++) {
        kalman_position_B.beta1_g *= 1.0020841800044864E-292;
      }

      kalman_position_B.atmp_f = kalman_position_B.beta1_g;
    } else {
      kalman_position_B.atmp_f = 1.0 / (kalman_position_B.b_A_m[21] -
        kalman_position_B.beta1_g);
      for (knt = 22; knt < 24; knt++) {
        kalman_position_B.b_A_m[knt] *= kalman_position_B.atmp_f;
      }

      kalman_position_B.atmp_f = kalman_position_B.beta1_g;
    }
  }

  kalman_position_B.b_A_m[21] = kalman_position_B.atmp_f;
  kalman_position_B.y_c[0] = kalman_position_B.b_A_m[0];
  for (knt = 1; knt + 1 < 5; knt++) {
    kalman_position_B.y_c[knt] = 0.0;
  }

  for (knt = 0; knt < 2; knt++) {
    kalman_position_B.y_c[knt + 4] = kalman_position_B.b_A_m[knt + 6];
  }

  for (knt = 2; knt + 1 < 5; knt++) {
    kalman_position_B.y_c[knt + 4] = 0.0;
  }

  for (knt = 0; knt < 3; knt++) {
    kalman_position_B.y_c[knt + 8] = kalman_position_B.b_A_m[knt + 12];
  }

  for (knt = 3; knt + 1 < 5; knt++) {
    kalman_position_B.y_c[knt + 8] = 0.0;
  }

  for (knt = 0; knt < 4; knt++) {
    kalman_position_B.y_c[knt + 12] = kalman_position_B.b_A_m[knt + 18];
  }

  for (knt = 0; knt < 4; knt++) {
    lastc = knt << 2;
    S[lastc] = kalman_position_B.y_c[knt];
    S[lastc + 1] = kalman_position_B.y_c[knt + 4];
    S[lastc + 2] = kalman_position_B.y_c[knt + 8];
    S[lastc + 3] = kalman_position_B.y_c[knt + 12];
  }
}

/* Function for MATLAB Function: '<S4>/Predict' */
static real_T kalman_position_xnrm2_d(int32_T n, const real_T x[32], int32_T ix0)
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  int32_T k;
  int32_T kend;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = fabs(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S4>/Predict' */
static void kalman_position_xgemv_l(int32_T m, int32_T n, const real_T A[32],
  int32_T ia0, const real_T x[32], int32_T ix0, real_T y[4])
{
  real_T c;
  int32_T b;
  int32_T b_iy;
  int32_T d;
  int32_T ia;
  int32_T iac;
  int32_T ix;
  if ((m != 0) && (n != 0)) {
    for (b_iy = 0; b_iy < n; b_iy++) {
      y[b_iy] = 0.0;
    }

    b_iy = 0;
    b = ((n - 1) << 3) + ia0;
    for (iac = ia0; iac <= b; iac += 8) {
      ix = ix0;
      c = 0.0;
      d = (iac + m) - 1;
      for (ia = iac; ia <= d; ia++) {
        c += A[ia - 1] * x[ix - 1];
        ix++;
      }

      y[b_iy] += c;
      b_iy++;
    }
  }
}

/* Function for MATLAB Function: '<S4>/Predict' */
static void kalman_position_xgerc_i(int32_T m, int32_T n, real_T alpha1, int32_T
  ix0, const real_T y[4], real_T A[32], int32_T ia0)
{
  real_T temp;
  int32_T b;
  int32_T ijA;
  int32_T ix;
  int32_T j;
  int32_T jA;
  int32_T jy;
  if (!(alpha1 == 0.0)) {
    jA = ia0 - 1;
    jy = 0;
    for (j = 0; j < n; j++) {
      if (y[jy] != 0.0) {
        temp = y[jy] * alpha1;
        ix = ix0;
        ijA = jA;
        b = m + jA;
        while (ijA + 1 <= b) {
          A[ijA] += A[ix - 1] * temp;
          ix++;
          ijA++;
        }
      }

      jy++;
      jA += 8;
    }
  }
}

/* Function for MATLAB Function: '<S4>/Predict' */
static void kalman_position_qrFactor_k(const real_T A[16], real_T S[16], const
  real_T Ns[16])
{
  int32_T aoffset;
  int32_T coffset;
  int32_T exitg1;
  int32_T knt;
  int32_T lastc;
  boolean_T exitg2;
  for (lastc = 0; lastc < 4; lastc++) {
    coffset = lastc << 2;
    for (knt = 0; knt < 4; knt++) {
      aoffset = knt << 2;
      kalman_position_B.y[coffset + knt] = ((S[aoffset + 1] * A[lastc + 4] +
        S[aoffset] * A[lastc]) + S[aoffset + 2] * A[lastc + 8]) + S[aoffset + 3]
        * A[lastc + 12];
    }
  }

  for (knt = 0; knt < 4; knt++) {
    lastc = knt << 3;
    coffset = knt << 2;
    kalman_position_B.b_A[lastc] = kalman_position_B.y[coffset];
    kalman_position_B.b_A[lastc + 4] = Ns[knt];
    kalman_position_B.b_A[lastc + 1] = kalman_position_B.y[coffset + 1];
    kalman_position_B.b_A[lastc + 5] = Ns[knt + 4];
    kalman_position_B.b_A[lastc + 2] = kalman_position_B.y[coffset + 2];
    kalman_position_B.b_A[lastc + 6] = Ns[knt + 8];
    kalman_position_B.b_A[lastc + 3] = kalman_position_B.y[coffset + 3];
    kalman_position_B.b_A[lastc + 7] = Ns[knt + 12];
    kalman_position_B.work[knt] = 0.0;
  }

  kalman_position_B.atmp = kalman_position_B.b_A[0];
  kalman_position_B.tau_idx_0 = 0.0;
  kalman_position_B.beta1 = kalman_position_xnrm2_d(7, kalman_position_B.b_A, 2);
  if (kalman_position_B.beta1 != 0.0) {
    kalman_position_B.beta1 = rt_hypotd_snf(kalman_position_B.b_A[0],
      kalman_position_B.beta1);
    if (kalman_position_B.b_A[0] >= 0.0) {
      kalman_position_B.beta1 = -kalman_position_B.beta1;
    }

    if (fabs(kalman_position_B.beta1) < 1.0020841800044864E-292) {
      knt = 0;
      do {
        knt++;
        for (lastc = 1; lastc < 8; lastc++) {
          kalman_position_B.b_A[lastc] *= 9.9792015476736E+291;
        }

        kalman_position_B.beta1 *= 9.9792015476736E+291;
        kalman_position_B.atmp *= 9.9792015476736E+291;
      } while ((fabs(kalman_position_B.beta1) < 1.0020841800044864E-292) && (knt
                < 20));

      kalman_position_B.beta1 = rt_hypotd_snf(kalman_position_B.atmp,
        kalman_position_xnrm2_d(7, kalman_position_B.b_A, 2));
      if (kalman_position_B.atmp >= 0.0) {
        kalman_position_B.beta1 = -kalman_position_B.beta1;
      }

      kalman_position_B.tau_idx_0 = (kalman_position_B.beta1 -
        kalman_position_B.atmp) / kalman_position_B.beta1;
      kalman_position_B.atmp = 1.0 / (kalman_position_B.atmp -
        kalman_position_B.beta1);
      for (lastc = 1; lastc < 8; lastc++) {
        kalman_position_B.b_A[lastc] *= kalman_position_B.atmp;
      }

      for (lastc = 0; lastc < knt; lastc++) {
        kalman_position_B.beta1 *= 1.0020841800044864E-292;
      }

      kalman_position_B.atmp = kalman_position_B.beta1;
    } else {
      kalman_position_B.tau_idx_0 = (kalman_position_B.beta1 -
        kalman_position_B.b_A[0]) / kalman_position_B.beta1;
      kalman_position_B.atmp = 1.0 / (kalman_position_B.b_A[0] -
        kalman_position_B.beta1);
      for (knt = 1; knt < 8; knt++) {
        kalman_position_B.b_A[knt] *= kalman_position_B.atmp;
      }

      kalman_position_B.atmp = kalman_position_B.beta1;
    }
  }

  kalman_position_B.b_A[0] = 1.0;
  if (kalman_position_B.tau_idx_0 != 0.0) {
    knt = 8;
    lastc = 7;
    while ((knt > 0) && (kalman_position_B.b_A[lastc] == 0.0)) {
      knt--;
      lastc--;
    }

    lastc = 3;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      coffset = ((lastc - 1) << 3) + 8;
      aoffset = coffset;
      do {
        exitg1 = 0;
        if (aoffset + 1 <= coffset + knt) {
          if (kalman_position_B.b_A[aoffset] != 0.0) {
            exitg1 = 1;
          } else {
            aoffset++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    knt = 0;
    lastc = 0;
  }

  if (knt > 0) {
    kalman_position_xgemv_l(knt, lastc, kalman_position_B.b_A, 9,
      kalman_position_B.b_A, 1, kalman_position_B.work);
    kalman_position_xgerc_i(knt, lastc, -kalman_position_B.tau_idx_0, 1,
      kalman_position_B.work, kalman_position_B.b_A, 9);
  }

  kalman_position_B.b_A[0] = kalman_position_B.atmp;
  kalman_position_B.atmp = kalman_position_B.b_A[9];
  kalman_position_B.tau_idx_0 = 0.0;
  kalman_position_B.beta1 = kalman_position_xnrm2_d(6, kalman_position_B.b_A, 11);
  if (kalman_position_B.beta1 != 0.0) {
    kalman_position_B.beta1 = rt_hypotd_snf(kalman_position_B.b_A[9],
      kalman_position_B.beta1);
    if (kalman_position_B.b_A[9] >= 0.0) {
      kalman_position_B.beta1 = -kalman_position_B.beta1;
    }

    if (fabs(kalman_position_B.beta1) < 1.0020841800044864E-292) {
      knt = 0;
      do {
        knt++;
        for (lastc = 10; lastc < 16; lastc++) {
          kalman_position_B.b_A[lastc] *= 9.9792015476736E+291;
        }

        kalman_position_B.beta1 *= 9.9792015476736E+291;
        kalman_position_B.atmp *= 9.9792015476736E+291;
      } while ((fabs(kalman_position_B.beta1) < 1.0020841800044864E-292) && (knt
                < 20));

      kalman_position_B.beta1 = rt_hypotd_snf(kalman_position_B.atmp,
        kalman_position_xnrm2_d(6, kalman_position_B.b_A, 11));
      if (kalman_position_B.atmp >= 0.0) {
        kalman_position_B.beta1 = -kalman_position_B.beta1;
      }

      kalman_position_B.tau_idx_0 = (kalman_position_B.beta1 -
        kalman_position_B.atmp) / kalman_position_B.beta1;
      kalman_position_B.atmp = 1.0 / (kalman_position_B.atmp -
        kalman_position_B.beta1);
      for (lastc = 10; lastc < 16; lastc++) {
        kalman_position_B.b_A[lastc] *= kalman_position_B.atmp;
      }

      for (lastc = 0; lastc < knt; lastc++) {
        kalman_position_B.beta1 *= 1.0020841800044864E-292;
      }

      kalman_position_B.atmp = kalman_position_B.beta1;
    } else {
      kalman_position_B.tau_idx_0 = (kalman_position_B.beta1 -
        kalman_position_B.b_A[9]) / kalman_position_B.beta1;
      kalman_position_B.atmp = 1.0 / (kalman_position_B.b_A[9] -
        kalman_position_B.beta1);
      for (knt = 10; knt < 16; knt++) {
        kalman_position_B.b_A[knt] *= kalman_position_B.atmp;
      }

      kalman_position_B.atmp = kalman_position_B.beta1;
    }
  }

  kalman_position_B.b_A[9] = 1.0;
  if (kalman_position_B.tau_idx_0 != 0.0) {
    knt = 7;
    lastc = 15;
    while ((knt > 0) && (kalman_position_B.b_A[lastc] == 0.0)) {
      knt--;
      lastc--;
    }

    lastc = 2;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      coffset = ((lastc - 1) << 3) + 17;
      aoffset = coffset;
      do {
        exitg1 = 0;
        if (aoffset + 1 <= coffset + knt) {
          if (kalman_position_B.b_A[aoffset] != 0.0) {
            exitg1 = 1;
          } else {
            aoffset++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    knt = 0;
    lastc = 0;
  }

  if (knt > 0) {
    kalman_position_xgemv_l(knt, lastc, kalman_position_B.b_A, 18,
      kalman_position_B.b_A, 10, kalman_position_B.work);
    kalman_position_xgerc_i(knt, lastc, -kalman_position_B.tau_idx_0, 10,
      kalman_position_B.work, kalman_position_B.b_A, 18);
  }

  kalman_position_B.b_A[9] = kalman_position_B.atmp;
  kalman_position_B.atmp = kalman_position_B.b_A[18];
  kalman_position_B.tau_idx_0 = 0.0;
  kalman_position_B.beta1 = kalman_position_xnrm2_d(5, kalman_position_B.b_A, 20);
  if (kalman_position_B.beta1 != 0.0) {
    kalman_position_B.beta1 = rt_hypotd_snf(kalman_position_B.b_A[18],
      kalman_position_B.beta1);
    if (kalman_position_B.b_A[18] >= 0.0) {
      kalman_position_B.beta1 = -kalman_position_B.beta1;
    }

    if (fabs(kalman_position_B.beta1) < 1.0020841800044864E-292) {
      knt = 0;
      do {
        knt++;
        for (lastc = 19; lastc < 24; lastc++) {
          kalman_position_B.b_A[lastc] *= 9.9792015476736E+291;
        }

        kalman_position_B.beta1 *= 9.9792015476736E+291;
        kalman_position_B.atmp *= 9.9792015476736E+291;
      } while ((fabs(kalman_position_B.beta1) < 1.0020841800044864E-292) && (knt
                < 20));

      kalman_position_B.beta1 = rt_hypotd_snf(kalman_position_B.atmp,
        kalman_position_xnrm2_d(5, kalman_position_B.b_A, 20));
      if (kalman_position_B.atmp >= 0.0) {
        kalman_position_B.beta1 = -kalman_position_B.beta1;
      }

      kalman_position_B.tau_idx_0 = (kalman_position_B.beta1 -
        kalman_position_B.atmp) / kalman_position_B.beta1;
      kalman_position_B.atmp = 1.0 / (kalman_position_B.atmp -
        kalman_position_B.beta1);
      for (lastc = 19; lastc < 24; lastc++) {
        kalman_position_B.b_A[lastc] *= kalman_position_B.atmp;
      }

      for (lastc = 0; lastc < knt; lastc++) {
        kalman_position_B.beta1 *= 1.0020841800044864E-292;
      }

      kalman_position_B.atmp = kalman_position_B.beta1;
    } else {
      kalman_position_B.tau_idx_0 = (kalman_position_B.beta1 -
        kalman_position_B.b_A[18]) / kalman_position_B.beta1;
      kalman_position_B.atmp = 1.0 / (kalman_position_B.b_A[18] -
        kalman_position_B.beta1);
      for (knt = 19; knt < 24; knt++) {
        kalman_position_B.b_A[knt] *= kalman_position_B.atmp;
      }

      kalman_position_B.atmp = kalman_position_B.beta1;
    }
  }

  kalman_position_B.b_A[18] = 1.0;
  if (kalman_position_B.tau_idx_0 != 0.0) {
    knt = 6;
    lastc = 23;
    while ((knt > 0) && (kalman_position_B.b_A[lastc] == 0.0)) {
      knt--;
      lastc--;
    }

    lastc = 1;
    aoffset = 26;
    do {
      exitg1 = 0;
      if (aoffset + 1 <= knt + 26) {
        if (kalman_position_B.b_A[aoffset] != 0.0) {
          exitg1 = 1;
        } else {
          aoffset++;
        }
      } else {
        lastc = 0;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    knt = 0;
    lastc = 0;
  }

  if (knt > 0) {
    kalman_position_xgemv_l(knt, lastc, kalman_position_B.b_A, 27,
      kalman_position_B.b_A, 19, kalman_position_B.work);
    kalman_position_xgerc_i(knt, lastc, -kalman_position_B.tau_idx_0, 19,
      kalman_position_B.work, kalman_position_B.b_A, 27);
  }

  kalman_position_B.b_A[18] = kalman_position_B.atmp;
  kalman_position_B.atmp = kalman_position_B.b_A[27];
  kalman_position_B.beta1 = kalman_position_xnrm2_d(4, kalman_position_B.b_A, 29);
  if (kalman_position_B.beta1 != 0.0) {
    kalman_position_B.beta1 = rt_hypotd_snf(kalman_position_B.b_A[27],
      kalman_position_B.beta1);
    if (kalman_position_B.b_A[27] >= 0.0) {
      kalman_position_B.beta1 = -kalman_position_B.beta1;
    }

    if (fabs(kalman_position_B.beta1) < 1.0020841800044864E-292) {
      knt = 0;
      do {
        knt++;
        for (lastc = 28; lastc < 32; lastc++) {
          kalman_position_B.b_A[lastc] *= 9.9792015476736E+291;
        }

        kalman_position_B.beta1 *= 9.9792015476736E+291;
        kalman_position_B.atmp *= 9.9792015476736E+291;
      } while ((fabs(kalman_position_B.beta1) < 1.0020841800044864E-292) && (knt
                < 20));

      kalman_position_B.beta1 = rt_hypotd_snf(kalman_position_B.atmp,
        kalman_position_xnrm2_d(4, kalman_position_B.b_A, 29));
      if (kalman_position_B.atmp >= 0.0) {
        kalman_position_B.beta1 = -kalman_position_B.beta1;
      }

      kalman_position_B.atmp = 1.0 / (kalman_position_B.atmp -
        kalman_position_B.beta1);
      for (lastc = 28; lastc < 32; lastc++) {
        kalman_position_B.b_A[lastc] *= kalman_position_B.atmp;
      }

      for (lastc = 0; lastc < knt; lastc++) {
        kalman_position_B.beta1 *= 1.0020841800044864E-292;
      }

      kalman_position_B.atmp = kalman_position_B.beta1;
    } else {
      kalman_position_B.atmp = 1.0 / (kalman_position_B.b_A[27] -
        kalman_position_B.beta1);
      for (knt = 28; knt < 32; knt++) {
        kalman_position_B.b_A[knt] *= kalman_position_B.atmp;
      }

      kalman_position_B.atmp = kalman_position_B.beta1;
    }
  }

  kalman_position_B.b_A[27] = kalman_position_B.atmp;
  kalman_position_B.y[0] = kalman_position_B.b_A[0];
  for (knt = 1; knt + 1 < 5; knt++) {
    kalman_position_B.y[knt] = 0.0;
  }

  for (knt = 0; knt < 2; knt++) {
    kalman_position_B.y[knt + 4] = kalman_position_B.b_A[knt + 8];
  }

  for (knt = 2; knt + 1 < 5; knt++) {
    kalman_position_B.y[knt + 4] = 0.0;
  }

  for (knt = 0; knt < 3; knt++) {
    kalman_position_B.y[knt + 8] = kalman_position_B.b_A[knt + 16];
  }

  for (knt = 3; knt + 1 < 5; knt++) {
    kalman_position_B.y[knt + 8] = 0.0;
  }

  for (knt = 0; knt < 4; knt++) {
    kalman_position_B.y[knt + 12] = kalman_position_B.b_A[knt + 24];
  }

  for (knt = 0; knt < 4; knt++) {
    lastc = knt << 2;
    S[lastc] = kalman_position_B.y[knt];
    S[lastc + 1] = kalman_position_B.y[knt + 4];
    S[lastc + 2] = kalman_position_B.y[knt + 8];
    S[lastc + 3] = kalman_position_B.y[knt + 12];
  }
}

/* Model step function */
void kalman_position_step(void)
{
  int32_T A_tmp;
  int32_T K_tmp;
  int32_T dHdx_tmp;
  int32_T j;

  /* Outputs for Enabled SubSystem: '<S1>/Correct1' incorporates:
   *  EnablePort: '<S2>/Enable'
   */
  /* MATLAB Function: '<S2>/Correct' incorporates:
   *  Constant: '<S1>/R1'
   *  DataStoreRead: '<S2>/Data Store ReadX'
   *  DataStoreWrite: '<S2>/Data Store WriteP'
   *  Inport: '<Root>/cos'
   *  Inport: '<Root>/sin'
   */
  kalman_position_B.z_idx_0_tmp = kalman_position_DW.x[2] * sin
    (kalman_position_DW.x[0]);
  kalman_position_B.z_idx_1_tmp = kalman_position_DW.x[3] * cos
    (kalman_position_DW.x[0]);
  for (j = 0; j < 4; j++) {
    kalman_position_B.z[0] = kalman_position_DW.x[0];
    kalman_position_B.z[2] = kalman_position_DW.x[2];
    kalman_position_B.z[3] = kalman_position_DW.x[3];
    kalman_position_B.epsilon = 1.4901161193847656E-8 * fabs
      (kalman_position_DW.x[j]);
    if ((kalman_position_B.epsilon <= 1.4901161193847656E-8) || rtIsNaN
        (kalman_position_B.epsilon)) {
      kalman_position_B.epsilon = 1.4901161193847656E-8;
    }

    kalman_position_B.z[j] = kalman_position_DW.x[j] + kalman_position_B.epsilon;
    dHdx_tmp = j << 1;
    kalman_position_B.dHdx[dHdx_tmp] = (kalman_position_B.z[2] * sin
      (kalman_position_B.z[0]) - kalman_position_B.z_idx_0_tmp) /
      kalman_position_B.epsilon;
    kalman_position_B.dHdx[dHdx_tmp + 1] = (kalman_position_B.z[3] * cos
      (kalman_position_B.z[0]) - kalman_position_B.z_idx_1_tmp) /
      kalman_position_B.epsilon;
  }

  kalman_position_qrFactor(kalman_position_B.dHdx, kalman_position_DW.P,
    kalman_position_ConstP.R1_Value, kalman_position_B.z);
  for (dHdx_tmp = 0; dHdx_tmp < 4; dHdx_tmp++) {
    for (j = 0; j < 4; j++) {
      K_tmp = (dHdx_tmp << 2) + j;
      kalman_position_B.dv[K_tmp] = 0.0;
      kalman_position_B.dv[K_tmp] += kalman_position_DW.P[j] *
        kalman_position_DW.P[dHdx_tmp];
      kalman_position_B.dv[K_tmp] += kalman_position_DW.P[j + 4] *
        kalman_position_DW.P[dHdx_tmp + 4];
      kalman_position_B.dv[K_tmp] += kalman_position_DW.P[j + 8] *
        kalman_position_DW.P[dHdx_tmp + 8];
      kalman_position_B.dv[K_tmp] += kalman_position_DW.P[j + 12] *
        kalman_position_DW.P[dHdx_tmp + 12];
    }
  }

  for (dHdx_tmp = 0; dHdx_tmp < 2; dHdx_tmp++) {
    for (j = 0; j < 4; j++) {
      K_tmp = (j << 1) + dHdx_tmp;
      kalman_position_B.K[K_tmp] = 0.0;
      kalman_position_B.K[K_tmp] += kalman_position_B.dv[j] *
        kalman_position_B.dHdx[dHdx_tmp];
      kalman_position_B.K[K_tmp] += kalman_position_B.dv[j + 4] *
        kalman_position_B.dHdx[dHdx_tmp + 2];
      kalman_position_B.K[K_tmp] += kalman_position_B.dv[j + 8] *
        kalman_position_B.dHdx[dHdx_tmp + 4];
      kalman_position_B.K[K_tmp] += kalman_position_B.dv[j + 12] *
        kalman_position_B.dHdx[dHdx_tmp + 6];
    }
  }

  for (dHdx_tmp = 0; dHdx_tmp < 4; dHdx_tmp++) {
    j = dHdx_tmp << 1;
    kalman_position_B.C[j] = kalman_position_B.K[j];
    kalman_position_B.C[j + 1] = kalman_position_B.K[j + 1];
  }

  kalman_position_trisolve(kalman_position_B.z, kalman_position_B.C);
  for (dHdx_tmp = 0; dHdx_tmp < 4; dHdx_tmp++) {
    j = dHdx_tmp << 1;
    kalman_position_B.b_C[j] = kalman_position_B.C[j];
    kalman_position_B.b_C[j + 1] = kalman_position_B.C[j + 1];
  }

  kalman_position_B.z_b[0] = kalman_position_B.z[0];
  kalman_position_B.z_b[1] = kalman_position_B.z[2];
  kalman_position_B.z_b[2] = kalman_position_B.z[1];
  kalman_position_B.z_b[3] = kalman_position_B.z[3];
  kalman_position_trisolve_c(kalman_position_B.z_b, kalman_position_B.b_C);
  for (dHdx_tmp = 0; dHdx_tmp < 2; dHdx_tmp++) {
    K_tmp = dHdx_tmp << 2;
    kalman_position_B.K[K_tmp] = kalman_position_B.b_C[dHdx_tmp];
    kalman_position_B.K[K_tmp + 1] = kalman_position_B.b_C[dHdx_tmp + 2];
    kalman_position_B.K[K_tmp + 2] = kalman_position_B.b_C[dHdx_tmp + 4];
    kalman_position_B.K[K_tmp + 3] = kalman_position_B.b_C[dHdx_tmp + 6];
  }

  for (dHdx_tmp = 0; dHdx_tmp < 8; dHdx_tmp++) {
    kalman_position_B.C[dHdx_tmp] = -kalman_position_B.K[dHdx_tmp];
  }

  for (dHdx_tmp = 0; dHdx_tmp < 4; dHdx_tmp++) {
    for (j = 0; j < 4; j++) {
      K_tmp = (dHdx_tmp << 2) + j;
      kalman_position_B.A[K_tmp] = 0.0;
      A_tmp = dHdx_tmp << 1;
      kalman_position_B.A[K_tmp] += kalman_position_B.dHdx[A_tmp] *
        kalman_position_B.C[j];
      kalman_position_B.A[K_tmp] += kalman_position_B.dHdx[A_tmp + 1] *
        kalman_position_B.C[j + 4];
    }
  }

  for (dHdx_tmp = 0; dHdx_tmp < 4; dHdx_tmp++) {
    kalman_position_B.C[dHdx_tmp] = 0.0;
    kalman_position_B.C[dHdx_tmp] += kalman_position_B.K[dHdx_tmp] * 0.1;
    kalman_position_B.epsilon = kalman_position_B.K[dHdx_tmp + 4];
    kalman_position_B.C[dHdx_tmp] += kalman_position_B.epsilon * 0.0;
    kalman_position_B.C[dHdx_tmp + 4] = 0.0;
    kalman_position_B.C[dHdx_tmp + 4] += kalman_position_B.K[dHdx_tmp] * 0.0;
    kalman_position_B.C[dHdx_tmp + 4] += kalman_position_B.epsilon * 0.1;
    K_tmp = (dHdx_tmp << 2) + dHdx_tmp;
    kalman_position_B.A[K_tmp]++;
  }

  kalman_position_qrFactor_j(kalman_position_B.A, kalman_position_DW.P,
    kalman_position_B.C);
  kalman_position_B.epsilon = kalman_position_U.sin_i -
    kalman_position_B.z_idx_0_tmp;
  kalman_position_B.z_idx_1_tmp = kalman_position_U.cos_a -
    kalman_position_B.z_idx_1_tmp;

  /* DataStoreWrite: '<S2>/Data Store WriteX' incorporates:
   *  DataStoreRead: '<S2>/Data Store ReadX'
   *  MATLAB Function: '<S2>/Correct'
   */
  for (dHdx_tmp = 0; dHdx_tmp < 4; dHdx_tmp++) {
    kalman_position_DW.x[dHdx_tmp] += kalman_position_B.K[dHdx_tmp + 4] *
      kalman_position_B.z_idx_1_tmp + kalman_position_B.K[dHdx_tmp] *
      kalman_position_B.epsilon;
  }

  /* End of DataStoreWrite: '<S2>/Data Store WriteX' */
  /* End of Outputs for SubSystem: '<S1>/Correct1' */

  /* Outport: '<Root>/phase1' incorporates:
   *  DataStoreRead: '<S3>/Data Store Read'
   */
  kalman_position_Y.phase1 = kalman_position_DW.x[0];

  /* Outport: '<Root>/speed1' incorporates:
   *  DataStoreRead: '<S3>/Data Store Read'
   */
  kalman_position_Y.speed1 = kalman_position_DW.x[1];

  /* Outputs for Atomic SubSystem: '<S1>/Predict' */
  /* MATLAB Function: '<S4>/Predict' incorporates:
   *  Constant: '<S1>/Q'
   *  DataStoreRead: '<S4>/Data Store ReadX'
   *  DataStoreWrite: '<S4>/Data Store WriteP'
   *  Inport: '<Root>/Ts'
   */
  kalman_position_B.dv[0] = 1.0;
  kalman_position_B.dv[4] = kalman_position_U.Ts;
  kalman_position_B.dv[8] = 0.0;
  kalman_position_B.dv[12] = 0.0;
  kalman_position_B.dv[1] = 0.0;
  kalman_position_B.dv[2] = 0.0;
  kalman_position_B.dv[3] = 0.0;
  kalman_position_B.dv[5] = 1.0;
  kalman_position_B.dv[6] = 0.0;
  kalman_position_B.dv[7] = 0.0;
  kalman_position_B.dv[9] = 0.0;
  kalman_position_B.dv[10] = 1.0;
  kalman_position_B.dv[11] = 0.0;
  kalman_position_B.dv[13] = 0.0;
  kalman_position_B.dv[14] = 0.0;
  kalman_position_B.dv[15] = 1.0;
  for (dHdx_tmp = 0; dHdx_tmp < 4; dHdx_tmp++) {
    kalman_position_B.z[dHdx_tmp] = ((kalman_position_B.dv[dHdx_tmp + 4] *
      kalman_position_DW.x[1] + kalman_position_B.dv[dHdx_tmp] *
      kalman_position_DW.x[0]) + kalman_position_B.dv[dHdx_tmp + 8] *
      kalman_position_DW.x[2]) + kalman_position_B.dv[dHdx_tmp + 12] *
      kalman_position_DW.x[3];
  }

  kalman_position_B.dv[0] = 1.0;
  kalman_position_B.dv[4] = kalman_position_U.Ts;
  kalman_position_B.dv[8] = 0.0;
  kalman_position_B.dv[12] = 0.0;
  kalman_position_B.dv[1] = 0.0;
  kalman_position_B.dv[2] = 0.0;
  kalman_position_B.dv[3] = 0.0;
  kalman_position_B.dv[5] = 1.0;
  kalman_position_B.dv[6] = 0.0;
  kalman_position_B.dv[7] = 0.0;
  kalman_position_B.dv[9] = 0.0;
  kalman_position_B.dv[10] = 1.0;
  kalman_position_B.dv[11] = 0.0;
  kalman_position_B.dv[13] = 0.0;
  kalman_position_B.dv[14] = 0.0;
  kalman_position_B.dv[15] = 1.0;
  for (j = 0; j < 4; j++) {
    kalman_position_B.z_b[0] = kalman_position_DW.x[0];
    kalman_position_B.z_b[1] = kalman_position_DW.x[1];
    kalman_position_B.z_b[2] = kalman_position_DW.x[2];
    kalman_position_B.z_b[3] = kalman_position_DW.x[3];
    kalman_position_B.epsilon = 1.4901161193847656E-8 * fabs
      (kalman_position_DW.x[j]);
    if ((kalman_position_B.epsilon <= 1.4901161193847656E-8) || rtIsNaN
        (kalman_position_B.epsilon)) {
      kalman_position_B.epsilon = 1.4901161193847656E-8;
    }

    kalman_position_B.z_b[j] = kalman_position_DW.x[j] +
      kalman_position_B.epsilon;
    for (dHdx_tmp = 0; dHdx_tmp < 4; dHdx_tmp++) {
      kalman_position_B.A[dHdx_tmp + (j << 2)] =
        ((((kalman_position_B.dv[dHdx_tmp + 4] * kalman_position_B.z_b[1] +
            kalman_position_B.dv[dHdx_tmp] * kalman_position_B.z_b[0]) +
           kalman_position_B.dv[dHdx_tmp + 8] * kalman_position_B.z_b[2]) +
          kalman_position_B.dv[dHdx_tmp + 12] * kalman_position_B.z_b[3]) -
         kalman_position_B.z[dHdx_tmp]) / kalman_position_B.epsilon;
    }
  }

  kalman_position_qrFactor_k(kalman_position_B.A, kalman_position_DW.P,
    kalman_position_ConstP.Q_Value);
  kalman_position_B.dv[0] = 1.0;
  kalman_position_B.dv[4] = kalman_position_U.Ts;
  kalman_position_B.dv[8] = 0.0;
  kalman_position_B.dv[12] = 0.0;
  kalman_position_B.dv[1] = 0.0;
  kalman_position_B.dv[2] = 0.0;
  kalman_position_B.dv[3] = 0.0;
  kalman_position_B.dv[5] = 1.0;
  kalman_position_B.dv[6] = 0.0;
  kalman_position_B.dv[7] = 0.0;
  kalman_position_B.dv[9] = 0.0;
  kalman_position_B.dv[10] = 1.0;
  kalman_position_B.dv[11] = 0.0;
  kalman_position_B.dv[13] = 0.0;
  kalman_position_B.dv[14] = 0.0;
  kalman_position_B.dv[15] = 1.0;
  for (dHdx_tmp = 0; dHdx_tmp < 4; dHdx_tmp++) {
    kalman_position_B.z[dHdx_tmp] = ((kalman_position_B.dv[dHdx_tmp + 4] *
      kalman_position_DW.x[1] + kalman_position_B.dv[dHdx_tmp] *
      kalman_position_DW.x[0]) + kalman_position_B.dv[dHdx_tmp + 8] *
      kalman_position_DW.x[2]) + kalman_position_B.dv[dHdx_tmp + 12] *
      kalman_position_DW.x[3];
  }

  /* End of MATLAB Function: '<S4>/Predict' */

  /* DataStoreWrite: '<S4>/Data Store WriteX' */
  kalman_position_DW.x[0] = kalman_position_B.z[0];
  kalman_position_DW.x[1] = kalman_position_B.z[1];
  kalman_position_DW.x[2] = kalman_position_B.z[2];
  kalman_position_DW.x[3] = kalman_position_B.z[3];

  /* End of Outputs for SubSystem: '<S1>/Predict' */
}

/* Model initialize function */
void kalman_position_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* Start for DataStoreMemory: '<S1>/DataStoreMemory - P' */
  memcpy(&kalman_position_DW.P[0],
         &kalman_position_ConstP.DataStoreMemoryP_InitialValue[0], sizeof(real_T)
         << 4U);

  /* Start for DataStoreMemory: '<S1>/DataStoreMemory - x' */
  kalman_position_DW.x[0] = 0.0;
  kalman_position_DW.x[1] = 0.0;
  kalman_position_DW.x[2] = 1.0;
  kalman_position_DW.x[3] = 1.0;
}

/* Model terminate function */
void kalman_position_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

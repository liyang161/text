/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: kalman_position.h
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

#ifndef RTW_HEADER_kalman_position_h_
#define RTW_HEADER_kalman_position_h_
#ifndef kalman_position_COMMON_INCLUDES_
#define kalman_position_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* kalman_position_COMMON_INCLUDES_ */

#include "kalman_position_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real_T b_A[32];
  real_T b_A_m[24];
  real_T A[16];
  real_T dv[16];
  real_T y[16];
  real_T y_c[16];
  real_T b_A_k[12];
  real_T dHdx[8];
  real_T K[8];
  real_T C[8];
  real_T b_C[8];
  real_T y_cx[8];
  real_T z[4];
  real_T z_b[4];
  real_T work[4];
  real_T work_p[4];
  real_T R[4];
  real_T work_c[2];
  real_T epsilon;
  real_T z_idx_0_tmp;
  real_T z_idx_1_tmp;
  real_T atmp;
  real_T beta1;
  real_T tau_idx_0;
  real_T atmp_f;
  real_T beta1_g;
  real_T tau_idx_0_g;
  real_T atmp_m;
  real_T beta1_n;
  real_T tau_idx_0_p;
} B_kalman_position_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T P[16];                        /* '<S1>/DataStoreMemory - P' */
  real_T x[4];                         /* '<S1>/DataStoreMemory - x' */
} DW_kalman_position_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: p.R{1}
   * Referenced by: '<S1>/R1'
   */
  real_T R1_Value[4];

  /* Expression: p.Q
   * Referenced by: '<S1>/Q'
   */
  real_T Q_Value[16];

  /* Expression: p.InitialCovariance
   * Referenced by: '<S1>/DataStoreMemory - P'
   */
  real_T DataStoreMemoryP_InitialValue[16];
} ConstP_kalman_position_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T sin_i;                        /* '<Root>/sin' */
  real_T cos_a;                        /* '<Root>/cos' */
  real_T Ts;                           /* '<Root>/Ts' */
} ExtU_kalman_position_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T phase1;                       /* '<Root>/phase1' */
  real_T speed1;                       /* '<Root>/speed1' */
} ExtY_kalman_position_T;

/* Real-time Model Data Structure */
struct tag_RTM_kalman_position_T {
  const char_T * volatile errorStatus;
};

/* Block signals (default storage) */
extern B_kalman_position_T kalman_position_B;

/* Block states (default storage) */
extern DW_kalman_position_T kalman_position_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_kalman_position_T kalman_position_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_kalman_position_T kalman_position_Y;

/* Constant parameters (default storage) */
extern const ConstP_kalman_position_T kalman_position_ConstP;

/* Model entry point functions */
extern void kalman_position_initialize(void);
extern void kalman_position_step(void);
extern void kalman_position_terminate(void);

/* Real-time Model object */
extern RT_MODEL_kalman_position_T *const kalman_position_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S2>/RegisterSimulinkFcn' : Unused code path elimination
 * Block '<S4>/RegisterSimulinkFcn' : Unused code path elimination
 * Block '<S1>/checkMeasurementFcn1Signals' : Unused code path elimination
 * Block '<S1>/checkStateTransitionFcnSignals' : Unused code path elimination
 * Block '<S1>/DataTypeConversion_Enable1' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_Q' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_R1' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_uMeas1' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_uState' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_y1' : Eliminate redundant data type conversion
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'kalman_position'
 * '<S1>'   : 'kalman_position/Extended Kalman Filter3'
 * '<S2>'   : 'kalman_position/Extended Kalman Filter3/Correct1'
 * '<S3>'   : 'kalman_position/Extended Kalman Filter3/Output'
 * '<S4>'   : 'kalman_position/Extended Kalman Filter3/Predict'
 * '<S5>'   : 'kalman_position/Extended Kalman Filter3/Correct1/Correct'
 * '<S6>'   : 'kalman_position/Extended Kalman Filter3/Output/MATLAB Function'
 * '<S7>'   : 'kalman_position/Extended Kalman Filter3/Predict/Predict'
 */
#endif                                 /* RTW_HEADER_kalman_position_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

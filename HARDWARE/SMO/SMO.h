/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: SMO.h
 *
 * Code generated for Simulink model 'SMO'.
 *
 * Model version                  : 1.60
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Tue Jun 27 02:56:08 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_SMO_h_
#define RTW_HEADER_SMO_h_
#ifndef SMO_COMMON_INCLUDES_
#define SMO_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* SMO_COMMON_INCLUDES_ */

#include "SMO_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UnitDelay_DSTATE;             /* '<S1>/Unit Delay' */
  real_T UnitDelay1_DSTATE;            /* '<S1>/Unit Delay1' */
} DW_SMO_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Ualpha;                       /* '<Root>/Ualpha' */
  real_T Ubeta;                        /* '<Root>/Ubeta' */
  real_T Ialpha;                       /* '<Root>/Ialpha' */
  real_T Ibeta;                        /* '<Root>/Ibeta' */
  real_T we_est;                       /* '<Root>/we_est' */
} ExtU_SMO_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Ealpha;                       /* '<Root>/Ealpha' */
  real_T Ealpha1;                      /* '<Root>/Ealpha1' */
} ExtY_SMO_T;

/* Real-time Model Data Structure */
struct tag_RTM_SMO_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_SMO_T SMO_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_SMO_T SMO_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_SMO_T SMO_Y;

/* Model entry point functions */
extern void SMO_initialize(void);
extern void SMO_step(void);
extern void SMO_terminate(void);

/* Real-time Model object */
extern RT_MODEL_SMO_T *const SMO_M;

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
 * '<Root>' : 'SMO'
 * '<S1>'   : 'SMO/Subsystem'
 */
#endif                                 /* RTW_HEADER_SMO_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

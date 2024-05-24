/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: FluxObserver.h
 *
 * Code generated for Simulink model 'FluxObserver'.
 *
 * Model version                  : 1.63
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Sun Jun 25 23:45:58 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_FluxObserver_h_
#define RTW_HEADER_FluxObserver_h_
#ifndef FluxObserver_COMMON_INCLUDES_
#define FluxObserver_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* FluxObserver_COMMON_INCLUDES_ */

#include "FluxObserver_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "zero_crossing_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Merge;                        /* '<S2>/Merge' */
} B_FluxObserver_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Delay_DSTATE;                 /* '<S4>/Delay' */
  real_T UnitDelay_DSTATE;             /* '<S18>/Unit Delay' */
  real_T Delay_DSTATE_l;               /* '<S3>/Delay' */
  real_T UnitDelay_DSTATE_g;           /* '<S13>/Unit Delay' */
} DW_FluxObserver_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState Delay_Reset_ZCE;          /* '<S4>/Delay' */
  ZCSigState Delay_Reset_ZCE_a;        /* '<S3>/Delay' */
} PrevZCX_FluxObserver_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Valpha;                       /* '<Root>/Valpha' */
  real_T Vbeta;                        /* '<Root>/Vbeta' */
  real_T Ialpha;                       /* '<Root>/Ialpha' */
  real_T Ibeta;                        /* '<Root>/Ibeta' */
} ExtU_FluxObserver_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T phase;                      /* '<Root>/phase' */
  real32_T flux;                       /* '<Root>/flux' */
  real32_T torque;                     /* '<Root>/torque' */
} ExtY_FluxObserver_T;

/* Real-time Model Data Structure */
struct tag_RTM_FluxObserver_T {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    struct {
      uint8_T TID[2];
    } TaskCounters;
  } Timing;
};

/* Block signals (default storage) */
extern B_FluxObserver_T FluxObserver_B;

/* Block states (default storage) */
extern DW_FluxObserver_T FluxObserver_DW;

/* Zero-crossing (trigger) state */
extern PrevZCX_FluxObserver_T FluxObserver_PrevZCX;

/* External inputs (root inport signals with default storage) */
extern ExtU_FluxObserver_T FluxObserver_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_FluxObserver_T FluxObserver_Y;

/* Model entry point functions */
extern void FluxObserver_initialize(void);
extern void FluxObserver_step(void);
extern void FluxObserver_terminate(void);

/* Real-time Model object */
extern RT_MODEL_FluxObserver_T *const FluxObserver_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Data Type Duplicate' : Unused code path elimination
 * Block '<S2>/Data Type Duplicate' : Unused code path elimination
 * Block '<S3>/Data Type Duplicate' : Unused code path elimination
 * Block '<S3>/Data Type Propagation' : Unused code path elimination
 * Block '<S13>/Data Type Duplicate' : Unused code path elimination
 * Block '<S4>/Data Type Duplicate' : Unused code path elimination
 * Block '<S4>/Data Type Propagation' : Unused code path elimination
 * Block '<S18>/Data Type Duplicate' : Unused code path elimination
 * Block '<S1>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion2' : Eliminate redundant data type conversion
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
 * '<Root>' : 'FluxObserver'
 * '<S1>'   : 'FluxObserver/Flux Observer'
 * '<S2>'   : 'FluxObserver/Flux Observer/atan2'
 * '<S3>'   : 'FluxObserver/Flux Observer/psiAlpha'
 * '<S4>'   : 'FluxObserver/Flux Observer/psiBeta'
 * '<S5>'   : 'FluxObserver/Flux Observer/atan2/Compare To Constant'
 * '<S6>'   : 'FluxObserver/Flux Observer/atan2/Compare To Constant1'
 * '<S7>'   : 'FluxObserver/Flux Observer/atan2/If Action Subsystem'
 * '<S8>'   : 'FluxObserver/Flux Observer/atan2/per Uint'
 * '<S9>'   : 'FluxObserver/Flux Observer/psiAlpha/IIR Filter'
 * '<S10>'  : 'FluxObserver/Flux Observer/psiAlpha/Scaling'
 * '<S11>'  : 'FluxObserver/Flux Observer/psiAlpha/IIR Filter/IIR Filter'
 * '<S12>'  : 'FluxObserver/Flux Observer/psiAlpha/IIR Filter/IIR Filter/High-pass'
 * '<S13>'  : 'FluxObserver/Flux Observer/psiAlpha/IIR Filter/IIR Filter/High-pass/IIR Low Pass Filter'
 * '<S14>'  : 'FluxObserver/Flux Observer/psiBeta/IIR Filter'
 * '<S15>'  : 'FluxObserver/Flux Observer/psiBeta/Scaling'
 * '<S16>'  : 'FluxObserver/Flux Observer/psiBeta/IIR Filter/IIR Filter'
 * '<S17>'  : 'FluxObserver/Flux Observer/psiBeta/IIR Filter/IIR Filter/High-pass'
 * '<S18>'  : 'FluxObserver/Flux Observer/psiBeta/IIR Filter/IIR Filter/High-pass/IIR Low Pass Filter'
 */
#endif                                 /* RTW_HEADER_FluxObserver_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: filter.h
 *
 * Code generated for Simulink model 'filter'.
 *
 * Model version                  : 1.3
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Wed Nov 22 21:01:54 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_filter_h_
#define RTW_HEADER_filter_h_
#ifndef filter_COMMON_INCLUDES_
#define filter_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* filter_COMMON_INCLUDES_ */

#include "filter_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Filter1_states[5];            /* '<Root>/Filter1' */
  real_T UD_DSTATE;                    /* '<S1>/UD' */
  real_T Filter2_states[5];            /* '<Root>/Filter2' */
} DW_filter_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Expression: [0.0399985281900833572 0.180984400031377168 0.344755318427351409 0.344755318427351409 0.180984400031377168 0.0399985281900833572])
   * Referenced by:
   *   '<Root>/Filter1'
   *   '<Root>/Filter2'
   */
  real_T pooled2[6];
} ConstP_filter_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T In;                           /* '<Root>/In' */
} ExtU_filter_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T speed;                        /* '<Root>/speed' */
  real_T acc;                          /* '<Root>/acc' */
} ExtY_filter_T;

/* Real-time Model Data Structure */
struct tag_RTM_filter_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_filter_T filter_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_filter_T filter_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_filter_T filter_Y;

/* Constant parameters (default storage) */
extern const ConstP_filter_T filter_ConstP;

/* Model entry point functions */
extern void filter_initialize(void);
extern void filter_step(void);
extern void filter_terminate(void);

/* Real-time Model object */
extern RT_MODEL_filter_T *const filter_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Data Type Duplicate' : Unused code path elimination
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
 * '<Root>' : 'filter'
 * '<S1>'   : 'filter/Discrete Derivative'
 */
#endif                                 /* RTW_HEADER_filter_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

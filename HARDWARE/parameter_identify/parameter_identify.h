/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: parameter_identify.h
 *
 * Code generated for Simulink model 'parameter_identify'.
 *
 * Model version                  : 1.66
 * Simulink Coder version         : 9.7 (R2022a) 13-Nov-2021
 * C/C++ source code generated on : Fri Jun 23 02:24:42 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_parameter_identify_h_
#define RTW_HEADER_parameter_identify_h_
#ifndef parameter_identify_COMMON_INCLUDES_
#define parameter_identify_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* parameter_identify_COMMON_INCLUDES_ */

#include "parameter_identify_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T UD_DSTATE;                    /* '<S5>/UD' */
  real_T Filter_FILT_STATES[10];       /* '<S2>/Filter' */
  real_T UD_DSTATE_c;                  /* '<S6>/UD' */
  real_T Filter1_FILT_STATES[10];      /* '<S2>/Filter1' */
  real_T UD_DSTATE_d;                  /* '<S3>/UD' */
  real_T Filter_FILT_STATES_d[10];     /* '<S1>/Filter' */
  real_T P_past[9];                    /* '<S1>/Data Store Memory' */
  real_T theta_past[3];                /* '<S1>/Data Store Memory1' */
  real_T P_past_l[9];                  /* '<S2>/Data Store Memory' */
  real_T theta_past_k[3];              /* '<S2>/Data Store Memory1' */
} DW_parameter_identify_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Expression: 1e4 * eye(3))
   * Referenced by:
   *   '<S1>/Data Store Memory'
   *   '<S2>/Data Store Memory'
   */
  real_T pooled7[9];
} ConstP_parameter_identify_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T id;                           /* '<Root>/id' */
  real_T iq;                           /* '<Root>/iq' */
  real_T w;                            /* '<Root>/w' */
  real_T ud;                           /* '<Root>/ud' */
  real_T uq;                           /* '<Root>/uq' */
} ExtU_parameter_identify_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T R;                            /* '<Root>/R' */
  real_T Ke;                           /* '<Root>/Ke' */
  real_T Ld;                           /* '<Root>/Ld' */
  real_T R1;                           /* '<Root>/R1' */
  real_T Ke1;                          /* '<Root>/Ke1' */
  real_T Ld1;                          /* '<Root>/Ld1' */
} ExtY_parameter_identify_T;

/* Real-time Model Data Structure */
struct tag_RTM_parameter_identify_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_parameter_identify_T parameter_identify_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_parameter_identify_T parameter_identify_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_parameter_identify_T parameter_identify_Y;

/* Constant parameters (default storage) */
extern const ConstP_parameter_identify_T parameter_identify_ConstP;

/* Model entry point functions */
extern void parameter_identify_initialize(void);
extern void parameter_identify_step(void);
extern void parameter_identify_terminate(void);

/* Real-time Model object */
extern RT_MODEL_parameter_identify_T *const parameter_identify_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S3>/Data Type Duplicate' : Unused code path elimination
 * Block '<S5>/Data Type Duplicate' : Unused code path elimination
 * Block '<S6>/Data Type Duplicate' : Unused code path elimination
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
 * '<Root>' : 'parameter_identify'
 * '<S1>'   : 'parameter_identify/Subsystem'
 * '<S2>'   : 'parameter_identify/Subsystem3'
 * '<S3>'   : 'parameter_identify/Subsystem/Discrete Derivative1'
 * '<S4>'   : 'parameter_identify/Subsystem/MATLAB Function'
 * '<S5>'   : 'parameter_identify/Subsystem3/Discrete Derivative1'
 * '<S6>'   : 'parameter_identify/Subsystem3/Discrete Derivative2'
 * '<S7>'   : 'parameter_identify/Subsystem3/MATLAB Function'
 */
#endif                                 /* RTW_HEADER_parameter_identify_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

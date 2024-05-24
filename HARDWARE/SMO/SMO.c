/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: SMO.c
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

#include "SMO.h"
#include "rtwtypes.h"

/* Block states (default storage) */
DW_SMO_T SMO_DW;

/* External inputs (root inport signals with default storage) */
ExtU_SMO_T SMO_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_SMO_T SMO_Y;

/* Real-time model */
static RT_MODEL_SMO_T SMO_M_;
RT_MODEL_SMO_T *const SMO_M = &SMO_M_;

/* Model step function */
void SMO_step(void)
{
  real_T rtb_Gain3;
  real_T rtb_Product;
  real_T rtb_Product1;
  real_T u0;

  /* Sum: '<S1>/Sum' incorporates:
   *  Inport: '<Root>/Ialpha'
   *  UnitDelay: '<S1>/Unit Delay'
   */
  u0 = SMO_DW.UnitDelay_DSTATE - SMO_U.Ialpha;

  /* Saturate: '<S1>/Saturation1' */
  if (u0 > 10.0) {
    u0 = 10.0;
  } else if (u0 < -10.0) {
    u0 = -10.0;
  }

  /* End of Saturate: '<S1>/Saturation1' */

  /* Gain: '<S1>/Lsw1' incorporates:
   *  Gain: '<S1>/Lsw'
   */
  SMO_Y.Ealpha1 = 2.0 * u0;

  /* Outport: '<Root>/Ealpha' */
  SMO_Y.Ealpha = SMO_Y.Ealpha1;

  /* Gain: '<S1>/Gain3' */
  rtb_Gain3 = 1.1560693641618498 * SMO_Y.Ealpha1;

  /* Gain: '<S1>/Lsw1' incorporates:
   *  Gain: '<S1>/Gain7'
   *  Inport: '<Root>/we_est'
   */
  SMO_Y.Ealpha1 = 0.0 * SMO_U.we_est;

  /* Product: '<S1>/Product' incorporates:
   *  UnitDelay: '<S1>/Unit Delay1'
   */
  rtb_Product = SMO_Y.Ealpha1 * SMO_DW.UnitDelay1_DSTATE;

  /* Product: '<S1>/Product1' incorporates:
   *  UnitDelay: '<S1>/Unit Delay'
   */
  rtb_Product1 = SMO_Y.Ealpha1 * SMO_DW.UnitDelay_DSTATE;

  /* Sum: '<S1>/Sum1' incorporates:
   *  Inport: '<Root>/Ibeta'
   *  UnitDelay: '<S1>/Unit Delay1'
   */
  u0 = SMO_DW.UnitDelay1_DSTATE - SMO_U.Ibeta;

  /* Saturate: '<S1>/Saturation' */
  if (u0 > 10.0) {
    u0 = 10.0;
  } else if (u0 < -10.0) {
    u0 = -10.0;
  }

  /* End of Saturate: '<S1>/Saturation' */

  /* Gain: '<S1>/Lsw1' */
  SMO_Y.Ealpha1 = 2.0 * u0;

  /* Sum: '<S1>/Add1' incorporates:
   *  Gain: '<S1>/Gain4'
   *  Gain: '<S1>/Gain5'
   *  Gain: '<S1>/Gain6'
   *  Inport: '<Root>/Ubeta'
   *  UnitDelay: '<S1>/Unit Delay1'
   */
  SMO_DW.UnitDelay1_DSTATE = (((1.1560693641618498 * SMO_U.Ubeta +
    SMO_DW.UnitDelay1_DSTATE) - 1.2716763005780349 * SMO_DW.UnitDelay1_DSTATE) -
    1.1560693641618498 * SMO_Y.Ealpha1) + rtb_Product1;

  /* Update for UnitDelay: '<S1>/Unit Delay' incorporates:
   *  Gain: '<S1>/Gain'
   *  Gain: '<S1>/Gain1'
   *  Inport: '<Root>/Ualpha'
   *  Sum: '<S1>/Add'
   */
  SMO_DW.UnitDelay_DSTATE = (((1.1560693641618498 * SMO_U.Ualpha +
    SMO_DW.UnitDelay_DSTATE) - 1.2716763005780349 * SMO_DW.UnitDelay_DSTATE) -
    rtb_Gain3) - rtb_Product;
}

/* Model initialize function */
void SMO_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void SMO_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

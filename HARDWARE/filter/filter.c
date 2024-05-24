/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: filter.c
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

#include "filter.h"
#include "rtwtypes.h"

/* Block states (default storage) */
DW_filter_T filter_DW;

/* External inputs (root inport signals with default storage) */
ExtU_filter_T filter_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_filter_T filter_Y;

/* Real-time model */
static RT_MODEL_filter_T filter_M_;
RT_MODEL_filter_T *const filter_M = &filter_M_;

/* Model step function */
void filter_step(void)
{
  real_T acc1;
  real_T rtb_TSamp;
  real_T zCurr;
  real_T zNext;
  int32_T n;

  /* DiscreteFir: '<Root>/Filter1' incorporates:
   *  Inport: '<Root>/In'
   */
  acc1 = 0.0;

  /* load input sample */
  zNext = filter_U.In;
  for (n = 0; n < 5; n++) {
    /* shift state */
    zCurr = zNext;
    zNext = filter_DW.Filter1_states[n];
    filter_DW.Filter1_states[n] = zCurr;

    /* compute one tap */
    acc1 += filter_ConstP.pooled2[n] * zCurr;
  }

  /* DiscreteFir: '<Root>/Filter1' */
  /* compute last tap */
  /* store output sample */
  filter_Y.speed = filter_ConstP.pooled2[n] * zNext + acc1;

  /* SampleTimeMath: '<S1>/TSamp'
   *
   * About '<S1>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp = filter_Y.speed * 50.0;

  /* DiscreteFir: '<Root>/Filter2' incorporates:
   *  Sum: '<S1>/Diff'
   *  UnitDelay: '<S1>/UD'
   *
   * Block description for '<S1>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S1>/UD':
   *
   *  Store in Global RAM
   */
  acc1 = 0.0;

  /* load input sample */
  zNext = rtb_TSamp - filter_DW.UD_DSTATE;
  for (n = 0; n < 5; n++) {
    /* shift state */
    zCurr = zNext;
    zNext = filter_DW.Filter2_states[n];
    filter_DW.Filter2_states[n] = zCurr;

    /* compute one tap */
    acc1 += filter_ConstP.pooled2[n] * zCurr;
  }

  /* Outport: '<Root>/acc' incorporates:
   *  DiscreteFir: '<Root>/Filter2'
   */
  /* compute last tap */
  /* store output sample */
  filter_Y.acc = filter_ConstP.pooled2[n] * zNext + acc1;

  /* Update for UnitDelay: '<S1>/UD'
   *
   * Block description for '<S1>/UD':
   *
   *  Store in Global RAM
   */
  filter_DW.UD_DSTATE = rtb_TSamp;
}

/* Model initialize function */
void filter_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void filter_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: FluxObserver.c
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

#include "FluxObserver.h"
#include "rtwtypes.h"
#include <math.h>
#include "FluxObserver_private.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"

/* Block signals (default storage) */
B_FluxObserver_T FluxObserver_B;

/* Block states (default storage) */
DW_FluxObserver_T FluxObserver_DW;

/* Previous zero-crossings (trigger) states */
PrevZCX_FluxObserver_T FluxObserver_PrevZCX;

/* External inputs (root inport signals with default storage) */
ExtU_FluxObserver_T FluxObserver_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_FluxObserver_T FluxObserver_Y;

/* Real-time model */
static RT_MODEL_FluxObserver_T FluxObserver_M_;
RT_MODEL_FluxObserver_T *const FluxObserver_M = &FluxObserver_M_;
static void rate_scheduler(void);

/*
 *         This function updates active task flag for each subrate.
 *         The function is called at model base rate, hence the
 *         generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (FluxObserver_M->Timing.TaskCounters.TID[1])++;
  if ((FluxObserver_M->Timing.TaskCounters.TID[1]) > 1) {/* Sample time: [0.02s, 0.0s] */
    FluxObserver_M->Timing.TaskCounters.TID[1] = 0;
  }
}

real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    int32_T u0_0;
    int32_T u1_0;
    if (u0 > 0.0F) {
      u0_0 = 1;
    } else {
      u0_0 = -1;
    }

    if (u1 > 0.0F) {
      u1_0 = 1;
    } else {
      u1_0 = -1;
    }

    y = atan2f((real32_T)u0_0, (real32_T)u1_0);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = atan2f(u0, u1);
  }

  return y;
}

/* Model step function */
void FluxObserver_step(void)
{
  if (FluxObserver_M->Timing.TaskCounters.TID[1] == 0) {
    real_T rtb_Sum1;
    real_T rtb_Sum1_l;

    /* Delay: '<S4>/Delay' */
    if (FluxObserver_PrevZCX.Delay_Reset_ZCE == POS_ZCSIG) {
      FluxObserver_DW.Delay_DSTATE = 0.0;
    }

    FluxObserver_PrevZCX.Delay_Reset_ZCE = 0U;

    /* Sum: '<S4>/Sum' incorporates:
     *  Delay: '<S4>/Delay'
     *  Gain: '<S15>/ScalingR'
     *  Gain: '<S4>/Gain'
     *  Inport: '<Root>/Ibeta'
     *  Inport: '<Root>/Vbeta'
     *  Sum: '<S4>/Sum3'
     */
    FluxObserver_DW.Delay_DSTATE += (FluxObserver_U.Vbeta - 1.088 *
      FluxObserver_U.Ibeta) * 0.019999999552965164;

    /* Sum: '<S4>/Sum1' incorporates:
     *  Delay: '<S4>/Delay'
     *  Gain: '<S15>/ScalingL'
     *  Inport: '<Root>/Ibeta'
     */
    rtb_Sum1 = FluxObserver_DW.Delay_DSTATE - 0.0172 * FluxObserver_U.Ibeta;

    /* Sum: '<S18>/Add1' incorporates:
     *  Constant: '<S18>/Filter_Constant'
     *  Constant: '<S18>/One'
     *  Product: '<S18>/Product'
     *  Product: '<S18>/Product1'
     *  UnitDelay: '<S18>/Unit Delay'
     */
    FluxObserver_DW.UnitDelay_DSTATE = rtb_Sum1 * 0.28591946496621445 +
      0.71408053503378555 * FluxObserver_DW.UnitDelay_DSTATE;

    /* Switch: '<S4>/Switch' incorporates:
     *  Sum: '<S17>/Sum'
     *  UnitDelay: '<S18>/Unit Delay'
     */
    rtb_Sum1 -= FluxObserver_DW.UnitDelay_DSTATE;

    /* Delay: '<S3>/Delay' */
    if (FluxObserver_PrevZCX.Delay_Reset_ZCE_a == POS_ZCSIG) {
      FluxObserver_DW.Delay_DSTATE_l = 0.0;
    }

    FluxObserver_PrevZCX.Delay_Reset_ZCE_a = 0U;

    /* Sum: '<S3>/Sum' incorporates:
     *  Delay: '<S3>/Delay'
     *  Gain: '<S10>/ScalingR'
     *  Gain: '<S3>/Gain'
     *  Inport: '<Root>/Ialpha'
     *  Inport: '<Root>/Valpha'
     *  Sum: '<S3>/Sum3'
     */
    FluxObserver_DW.Delay_DSTATE_l += (FluxObserver_U.Valpha - 1.088 *
      FluxObserver_U.Ialpha) * 0.019999999552965164;

    /* Sum: '<S3>/Sum1' incorporates:
     *  Delay: '<S3>/Delay'
     *  Gain: '<S10>/ScalingL'
     *  Inport: '<Root>/Ialpha'
     */
    rtb_Sum1_l = FluxObserver_DW.Delay_DSTATE_l - 0.0172 * FluxObserver_U.Ialpha;

    /* Sum: '<S13>/Add1' incorporates:
     *  Constant: '<S13>/Filter_Constant'
     *  Constant: '<S13>/One'
     *  Product: '<S13>/Product'
     *  Product: '<S13>/Product1'
     *  UnitDelay: '<S13>/Unit Delay'
     */
    FluxObserver_DW.UnitDelay_DSTATE_g = rtb_Sum1_l * 0.28591946496621445 +
      0.71408053503378555 * FluxObserver_DW.UnitDelay_DSTATE_g;

    /* Switch: '<S3>/Switch' incorporates:
     *  Sum: '<S12>/Sum'
     *  UnitDelay: '<S13>/Unit Delay'
     */
    rtb_Sum1_l -= FluxObserver_DW.UnitDelay_DSTATE_g;

    /* Outport: '<Root>/flux' incorporates:
     *  Math: '<S1>/Square'
     *  Math: '<S1>/Square1'
     *  Sqrt: '<S1>/Sqrt'
     *  Sum: '<S1>/Sum'
     */
    FluxObserver_Y.flux = (real32_T)sqrt(rtb_Sum1_l * rtb_Sum1_l + rtb_Sum1 *
      rtb_Sum1);

    /* Outport: '<Root>/torque' incorporates:
     *  Gain: '<S1>/TorqueGain'
     *  Inport: '<Root>/Ialpha'
     *  Inport: '<Root>/Ibeta'
     *  Product: '<S1>/Product'
     *  Product: '<S1>/Product1'
     *  Sum: '<S1>/Sum1'
     */
    FluxObserver_Y.torque = (real32_T)((rtb_Sum1_l * FluxObserver_U.Ibeta -
      rtb_Sum1 * FluxObserver_U.Ialpha) * 1.5);

    /* Outputs for Atomic SubSystem: '<S1>/atan2' */
    /* Outputs for Enabled SubSystem: '<S2>/per Uint' incorporates:
     *  EnablePort: '<S8>/Enable'
     */
    /* Gain: '<S8>/Gain' incorporates:
     *  DataTypeConversion: '<S2>/Data Type Conversion'
     *  DataTypeConversion: '<S2>/Data Type Conversion1'
     *  DataTypeConversion: '<S2>/Data Type Conversion2'
     *  Trigonometry: '<S2>/Atan2'
     */
    FluxObserver_B.Merge = 0.15915494309189535 * rt_atan2f_snf((real32_T)
      rtb_Sum1, (real32_T)rtb_Sum1_l);

    /* Switch: '<S8>/Switch' */
    if (!(FluxObserver_B.Merge >= 0.0)) {
      /* Gain: '<S8>/Gain' incorporates:
       *  Bias: '<S8>/Bias'
       *  Merge: '<S2>/Merge'
       */
      FluxObserver_B.Merge++;
    }

    /* End of Switch: '<S8>/Switch' */
    /* End of Outputs for SubSystem: '<S2>/per Uint' */

    /* Outport: '<Root>/phase' incorporates:
     *  AlgorithmDescriptorDelegate generated from: '<S2>/a16'
     *  Gain: '<S1>/PositionGain'
     */
    FluxObserver_Y.phase = (real32_T)(6.2831853071795862 * FluxObserver_B.Merge);

    /* End of Outputs for SubSystem: '<S1>/atan2' */
  }

  rate_scheduler();
}

/* Model initialize function */
void FluxObserver_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));
  FluxObserver_PrevZCX.Delay_Reset_ZCE = UNINITIALIZED_ZCSIG;
  FluxObserver_PrevZCX.Delay_Reset_ZCE_a = UNINITIALIZED_ZCSIG;
}

/* Model terminate function */
void FluxObserver_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

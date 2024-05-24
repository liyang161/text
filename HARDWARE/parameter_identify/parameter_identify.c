/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: parameter_identify.c
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

#include "parameter_identify.h"
#include <string.h>
#include "rtwtypes.h"

/* Block states (default storage) */
DW_parameter_identify_T parameter_identify_DW;

/* External inputs (root inport signals with default storage) */
ExtU_parameter_identify_T parameter_identify_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_parameter_identify_T parameter_identify_Y;

/* Real-time model */
static RT_MODEL_parameter_identify_T parameter_identify_M_;
RT_MODEL_parameter_identify_T *const parameter_identify_M =
  &parameter_identify_M_;

/* Model step function */
void parameter_identify_step(void)
{
  real_T tmp[9];
  real_T tmp_0[9];
  real_T xt_0[9];
  real_T theta_new[3];
  real_T xt[3];
  real_T denAccum;
  real_T rtb_Filter;
  real_T rtb_Filter1;
  real_T rtb_Filter_j;
  real_T rtb_TSamp;
  real_T rtb_TSamp_i_tmp;
  real_T xt_1;
  real_T xt_tmp_0;
  real_T xt_tmp_1;
  int32_T i;
  int32_T tmp_1;
  int32_T xt_tmp;

  /* SampleTimeMath: '<S5>/TSamp' incorporates:
   *  Inport: '<Root>/id'
   *
   * About '<S5>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp = parameter_identify_U.id * 100.0;

  /* S-Function (sdspbiquad): '<S2>/Filter' incorporates:
   *  Sum: '<S5>/Diff'
   *  UnitDelay: '<S5>/UD'
   *
   * Block description for '<S5>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S5>/UD':
   *
   *  Store in Global RAM
   */
  denAccum = ((rtb_TSamp - parameter_identify_DW.UD_DSTATE) *
              0.087450445578853356 - -1.4817841285386757 *
              parameter_identify_DW.Filter_FILT_STATES[0]) - 0.83158591085408906
    * parameter_identify_DW.Filter_FILT_STATES[1];
  rtb_Filter1 = (2.0 * parameter_identify_DW.Filter_FILT_STATES[0] + denAccum) +
    parameter_identify_DW.Filter_FILT_STATES[1];
  parameter_identify_DW.Filter_FILT_STATES[1] =
    parameter_identify_DW.Filter_FILT_STATES[0];
  parameter_identify_DW.Filter_FILT_STATES[0] = denAccum;
  denAccum = (0.0753771829242908 * rtb_Filter1 - -1.2772114832789294 *
              parameter_identify_DW.Filter_FILT_STATES[2]) - 0.5787202149760925 *
    parameter_identify_DW.Filter_FILT_STATES[3];
  rtb_Filter1 = (2.0 * parameter_identify_DW.Filter_FILT_STATES[2] + denAccum) +
    parameter_identify_DW.Filter_FILT_STATES[3];
  parameter_identify_DW.Filter_FILT_STATES[3] =
    parameter_identify_DW.Filter_FILT_STATES[2];
  parameter_identify_DW.Filter_FILT_STATES[2] = denAccum;
  denAccum = (0.0674552738890719 * rtb_Filter1 - -1.1429805025399011 *
              parameter_identify_DW.Filter_FILT_STATES[4]) - 0.41280159809618877
    * parameter_identify_DW.Filter_FILT_STATES[5];
  rtb_Filter1 = (2.0 * parameter_identify_DW.Filter_FILT_STATES[4] + denAccum) +
    parameter_identify_DW.Filter_FILT_STATES[5];
  parameter_identify_DW.Filter_FILT_STATES[5] =
    parameter_identify_DW.Filter_FILT_STATES[4];
  parameter_identify_DW.Filter_FILT_STATES[4] = denAccum;
  denAccum = (0.062669960238802266 * rtb_Filter1 - -1.0618968468751018 *
              parameter_identify_DW.Filter_FILT_STATES[6]) - 0.31257668783031084
    * parameter_identify_DW.Filter_FILT_STATES[7];
  rtb_Filter1 = (2.0 * parameter_identify_DW.Filter_FILT_STATES[6] + denAccum) +
    parameter_identify_DW.Filter_FILT_STATES[7];
  parameter_identify_DW.Filter_FILT_STATES[7] =
    parameter_identify_DW.Filter_FILT_STATES[6];
  parameter_identify_DW.Filter_FILT_STATES[6] = denAccum;
  denAccum = (0.060416680871365334 * rtb_Filter1 - -1.0237166685840595 *
              parameter_identify_DW.Filter_FILT_STATES[8]) - 0.26538339206952077
    * parameter_identify_DW.Filter_FILT_STATES[9];
  rtb_Filter = (2.0 * parameter_identify_DW.Filter_FILT_STATES[8] + denAccum) +
    parameter_identify_DW.Filter_FILT_STATES[9];
  parameter_identify_DW.Filter_FILT_STATES[9] =
    parameter_identify_DW.Filter_FILT_STATES[8];
  parameter_identify_DW.Filter_FILT_STATES[8] = denAccum;

  /* SampleTimeMath: '<S6>/TSamp' incorporates:
   *  Inport: '<Root>/iq'
   *  SampleTimeMath: '<S3>/TSamp'
   *
   * About '<S6>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *
   * About '<S3>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   */
  rtb_TSamp_i_tmp = parameter_identify_U.iq * 100.0;

  /* S-Function (sdspbiquad): '<S2>/Filter1' incorporates:
   *  SampleTimeMath: '<S6>/TSamp'
   *  Sum: '<S6>/Diff'
   *  UnitDelay: '<S6>/UD'
   *
   * About '<S6>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *
   * Block description for '<S6>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S6>/UD':
   *
   *  Store in Global RAM
   */
  denAccum = ((rtb_TSamp_i_tmp - parameter_identify_DW.UD_DSTATE_c) *
              0.087450445578853356 - -1.4817841285386757 *
              parameter_identify_DW.Filter1_FILT_STATES[0]) -
    0.83158591085408906 * parameter_identify_DW.Filter1_FILT_STATES[1];
  rtb_Filter1 = (2.0 * parameter_identify_DW.Filter1_FILT_STATES[0] + denAccum)
    + parameter_identify_DW.Filter1_FILT_STATES[1];
  parameter_identify_DW.Filter1_FILT_STATES[1] =
    parameter_identify_DW.Filter1_FILT_STATES[0];
  parameter_identify_DW.Filter1_FILT_STATES[0] = denAccum;
  denAccum = (0.0753771829242908 * rtb_Filter1 - -1.2772114832789294 *
              parameter_identify_DW.Filter1_FILT_STATES[2]) - 0.5787202149760925
    * parameter_identify_DW.Filter1_FILT_STATES[3];
  rtb_Filter1 = (2.0 * parameter_identify_DW.Filter1_FILT_STATES[2] + denAccum)
    + parameter_identify_DW.Filter1_FILT_STATES[3];
  parameter_identify_DW.Filter1_FILT_STATES[3] =
    parameter_identify_DW.Filter1_FILT_STATES[2];
  parameter_identify_DW.Filter1_FILT_STATES[2] = denAccum;
  denAccum = (0.0674552738890719 * rtb_Filter1 - -1.1429805025399011 *
              parameter_identify_DW.Filter1_FILT_STATES[4]) -
    0.41280159809618877 * parameter_identify_DW.Filter1_FILT_STATES[5];
  rtb_Filter1 = (2.0 * parameter_identify_DW.Filter1_FILT_STATES[4] + denAccum)
    + parameter_identify_DW.Filter1_FILT_STATES[5];
  parameter_identify_DW.Filter1_FILT_STATES[5] =
    parameter_identify_DW.Filter1_FILT_STATES[4];
  parameter_identify_DW.Filter1_FILT_STATES[4] = denAccum;
  denAccum = (0.062669960238802266 * rtb_Filter1 - -1.0618968468751018 *
              parameter_identify_DW.Filter1_FILT_STATES[6]) -
    0.31257668783031084 * parameter_identify_DW.Filter1_FILT_STATES[7];
  rtb_Filter1 = (2.0 * parameter_identify_DW.Filter1_FILT_STATES[6] + denAccum)
    + parameter_identify_DW.Filter1_FILT_STATES[7];
  parameter_identify_DW.Filter1_FILT_STATES[7] =
    parameter_identify_DW.Filter1_FILT_STATES[6];
  parameter_identify_DW.Filter1_FILT_STATES[6] = denAccum;
  denAccum = (0.060416680871365334 * rtb_Filter1 - -1.0237166685840595 *
              parameter_identify_DW.Filter1_FILT_STATES[8]) -
    0.26538339206952077 * parameter_identify_DW.Filter1_FILT_STATES[9];
  rtb_Filter1 = (2.0 * parameter_identify_DW.Filter1_FILT_STATES[8] + denAccum)
    + parameter_identify_DW.Filter1_FILT_STATES[9];
  parameter_identify_DW.Filter1_FILT_STATES[9] =
    parameter_identify_DW.Filter1_FILT_STATES[8];
  parameter_identify_DW.Filter1_FILT_STATES[8] = denAccum;

  /* MATLAB Function: '<S2>/MATLAB Function' incorporates:
   *  Inport: '<Root>/id'
   *  Inport: '<Root>/iq'
   *  Inport: '<Root>/ud'
   *  Inport: '<Root>/uq'
   *  Inport: '<Root>/w'
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  SignalConversion generated from: '<S7>/ SFunction '
   */
  xt[0] = parameter_identify_U.iq;
  xt_tmp_1 = parameter_identify_U.id * parameter_identify_U.w;
  xt[1] = xt_tmp_1 + rtb_Filter1;
  xt[2] = parameter_identify_U.w;
  denAccum = 0.0;
  xt_1 = 0.0;
  rtb_Filter1 = 0.0;
  for (i = 0; i < 3; i++) {
    rtb_Filter1 += xt[i] * parameter_identify_DW.theta_past_k[i];
    xt_0[3 * i] = parameter_identify_U.iq * xt[i];
    xt_tmp = 3 * i + 1;
    xt_tmp_0 = parameter_identify_DW.P_past_l[xt_tmp] * xt[1];
    xt_0[xt_tmp] = xt[1] * xt[i];
    xt_tmp = 3 * i + 2;
    xt_0[xt_tmp] = parameter_identify_U.w * xt[i];
    xt_tmp_0 = ((parameter_identify_DW.P_past_l[3 * i] * parameter_identify_U.iq
                 + xt_tmp_0) + parameter_identify_DW.P_past_l[xt_tmp] *
                parameter_identify_U.w) * xt[i];
    denAccum += xt_tmp_0;
    xt_1 += xt_tmp_0;
    theta_new[i] = (parameter_identify_DW.P_past_l[i + 3] * xt[1] +
                    parameter_identify_DW.P_past_l[i] * parameter_identify_U.iq)
      + parameter_identify_DW.P_past_l[i + 6] * parameter_identify_U.w;
  }

  rtb_Filter_j = parameter_identify_U.uq - rtb_Filter1;
  for (i = 0; i < 3; i++) {
    for (xt_tmp = 0; xt_tmp < 3; xt_tmp++) {
      tmp_1 = 3 * xt_tmp + i;
      tmp[tmp_1] = 0.0;
      tmp[tmp_1] += xt_0[3 * xt_tmp] * parameter_identify_DW.P_past_l[i];
      tmp[tmp_1] += xt_0[3 * xt_tmp + 1] * parameter_identify_DW.P_past_l[i + 3];
      tmp[tmp_1] += xt_0[3 * xt_tmp + 2] * parameter_identify_DW.P_past_l[i + 6];
    }

    for (xt_tmp = 0; xt_tmp < 3; xt_tmp++) {
      tmp_1 = 3 * xt_tmp + i;
      tmp_0[tmp_1] = parameter_identify_DW.P_past_l[tmp_1] -
        ((parameter_identify_DW.P_past_l[3 * xt_tmp + 1] * tmp[i + 3] +
          parameter_identify_DW.P_past_l[3 * xt_tmp] * tmp[i]) +
         parameter_identify_DW.P_past_l[3 * xt_tmp + 2] * tmp[i + 6]) /
        (denAccum + 1.0);
    }
  }

  memcpy(&parameter_identify_DW.P_past_l[0], &tmp_0[0], 9U * sizeof(real_T));
  xt[0] = parameter_identify_U.id;
  xt[1] = rtb_Filter - parameter_identify_U.iq * parameter_identify_U.w;
  xt[2] = 0.0;
  denAccum = 0.0;
  rtb_Filter1 = 0.0;
  rtb_Filter = 0.0;
  for (i = 0; i < 3; i++) {
    parameter_identify_DW.theta_past_k[i] += theta_new[i] / (xt_1 + 1.0) *
      rtb_Filter_j;
    xt_tmp_0 = ((parameter_identify_DW.P_past_l[3 * i + 1] * xt[1] +
                 parameter_identify_DW.P_past_l[3 * i] * parameter_identify_U.id)
                + parameter_identify_DW.P_past_l[3 * i + 2] * 0.0) * xt[i];
    denAccum += xt_tmp_0;
    rtb_Filter1 += xt_tmp_0;
    rtb_Filter += xt[i] * parameter_identify_DW.theta_past_k[i];
  }

  xt_1 = parameter_identify_U.ud - rtb_Filter;
  for (i = 0; i < 3; i++) {
    xt_0[3 * i] = parameter_identify_U.id * xt[i];
    xt_0[3 * i + 1] = xt[1] * xt[i];
    xt_0[3 * i + 2] = 0.0 * xt[i];
    theta_new[i] = ((parameter_identify_DW.P_past_l[i + 3] * xt[1] +
                     parameter_identify_DW.P_past_l[i] * parameter_identify_U.id)
                    + parameter_identify_DW.P_past_l[i + 6] * 0.0) /
      (rtb_Filter1 + 1.0) * xt_1 + parameter_identify_DW.theta_past_k[i];
  }

  for (i = 0; i < 3; i++) {
    for (xt_tmp = 0; xt_tmp < 3; xt_tmp++) {
      tmp_1 = 3 * xt_tmp + i;
      tmp[tmp_1] = 0.0;
      tmp[tmp_1] += xt_0[3 * xt_tmp] * parameter_identify_DW.P_past_l[i];
      tmp[tmp_1] += xt_0[3 * xt_tmp + 1] * parameter_identify_DW.P_past_l[i + 3];
      tmp[tmp_1] += xt_0[3 * xt_tmp + 2] * parameter_identify_DW.P_past_l[i + 6];
    }

    for (xt_tmp = 0; xt_tmp < 3; xt_tmp++) {
      tmp_1 = 3 * xt_tmp + i;
      tmp_0[tmp_1] = parameter_identify_DW.P_past_l[tmp_1] -
        ((parameter_identify_DW.P_past_l[3 * xt_tmp + 1] * tmp[i + 3] +
          parameter_identify_DW.P_past_l[3 * xt_tmp] * tmp[i]) +
         parameter_identify_DW.P_past_l[3 * xt_tmp + 2] * tmp[i + 6]) /
        (denAccum + 1.0);
    }
  }

  memcpy(&parameter_identify_DW.P_past_l[0], &tmp_0[0], 9U * sizeof(real_T));

  /* Outport: '<Root>/R' incorporates:
   *  MATLAB Function: '<S2>/MATLAB Function'
   */
  parameter_identify_Y.R = theta_new[0];

  /* Outport: '<Root>/Ke' incorporates:
   *  MATLAB Function: '<S2>/MATLAB Function'
   */
  parameter_identify_Y.Ke = theta_new[2];

  /* Outport: '<Root>/Ld' incorporates:
   *  MATLAB Function: '<S2>/MATLAB Function'
   */
  parameter_identify_Y.Ld = theta_new[1];

  /* S-Function (sdspbiquad): '<S1>/Filter' incorporates:
   *  Sum: '<S3>/Diff'
   *  UnitDelay: '<S3>/UD'
   *
   * Block description for '<S3>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S3>/UD':
   *
   *  Store in Global RAM
   */
  denAccum = ((rtb_TSamp_i_tmp - parameter_identify_DW.UD_DSTATE_d) *
              0.087450445578853356 - -1.4817841285386757 *
              parameter_identify_DW.Filter_FILT_STATES_d[0]) -
    0.83158591085408906 * parameter_identify_DW.Filter_FILT_STATES_d[1];
  rtb_Filter1 = (2.0 * parameter_identify_DW.Filter_FILT_STATES_d[0] + denAccum)
    + parameter_identify_DW.Filter_FILT_STATES_d[1];
  parameter_identify_DW.Filter_FILT_STATES_d[1] =
    parameter_identify_DW.Filter_FILT_STATES_d[0];
  parameter_identify_DW.Filter_FILT_STATES_d[0] = denAccum;
  denAccum = (0.0753771829242908 * rtb_Filter1 - -1.2772114832789294 *
              parameter_identify_DW.Filter_FILT_STATES_d[2]) -
    0.5787202149760925 * parameter_identify_DW.Filter_FILT_STATES_d[3];
  rtb_Filter1 = (2.0 * parameter_identify_DW.Filter_FILT_STATES_d[2] + denAccum)
    + parameter_identify_DW.Filter_FILT_STATES_d[3];
  parameter_identify_DW.Filter_FILT_STATES_d[3] =
    parameter_identify_DW.Filter_FILT_STATES_d[2];
  parameter_identify_DW.Filter_FILT_STATES_d[2] = denAccum;
  denAccum = (0.0674552738890719 * rtb_Filter1 - -1.1429805025399011 *
              parameter_identify_DW.Filter_FILT_STATES_d[4]) -
    0.41280159809618877 * parameter_identify_DW.Filter_FILT_STATES_d[5];
  rtb_Filter1 = (2.0 * parameter_identify_DW.Filter_FILT_STATES_d[4] + denAccum)
    + parameter_identify_DW.Filter_FILT_STATES_d[5];
  parameter_identify_DW.Filter_FILT_STATES_d[5] =
    parameter_identify_DW.Filter_FILT_STATES_d[4];
  parameter_identify_DW.Filter_FILT_STATES_d[4] = denAccum;
  denAccum = (0.062669960238802266 * rtb_Filter1 - -1.0618968468751018 *
              parameter_identify_DW.Filter_FILT_STATES_d[6]) -
    0.31257668783031084 * parameter_identify_DW.Filter_FILT_STATES_d[7];
  rtb_Filter1 = (2.0 * parameter_identify_DW.Filter_FILT_STATES_d[6] + denAccum)
    + parameter_identify_DW.Filter_FILT_STATES_d[7];
  parameter_identify_DW.Filter_FILT_STATES_d[7] =
    parameter_identify_DW.Filter_FILT_STATES_d[6];
  parameter_identify_DW.Filter_FILT_STATES_d[6] = denAccum;
  denAccum = (0.060416680871365334 * rtb_Filter1 - -1.0237166685840595 *
              parameter_identify_DW.Filter_FILT_STATES_d[8]) -
    0.26538339206952077 * parameter_identify_DW.Filter_FILT_STATES_d[9];
  rtb_Filter_j = (2.0 * parameter_identify_DW.Filter_FILT_STATES_d[8] + denAccum)
    + parameter_identify_DW.Filter_FILT_STATES_d[9];
  parameter_identify_DW.Filter_FILT_STATES_d[9] =
    parameter_identify_DW.Filter_FILT_STATES_d[8];
  parameter_identify_DW.Filter_FILT_STATES_d[8] = denAccum;

  /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
   *  Inport: '<Root>/iq'
   *  Inport: '<Root>/uq'
   *  Inport: '<Root>/w'
   */
  xt[0] = -parameter_identify_U.iq;
  xt[1] = -parameter_identify_U.w;
  xt[2] = parameter_identify_U.uq;
  denAccum = 0.0;
  xt_1 = 0.0;
  rtb_Filter1 = 0.0;
  for (i = 0; i < 3; i++) {
    /* MATLAB Function: '<S1>/MATLAB Function' */
    rtb_Filter = xt[i];

    /* MATLAB Function: '<S2>/MATLAB Function' */
    parameter_identify_DW.theta_past_k[i] = theta_new[i];

    /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
     *  Inport: '<Root>/iq'
     *  Inport: '<Root>/uq'
     *  Inport: '<Root>/w'
     */
    xt_tmp_0 = ((parameter_identify_DW.P_past[3 * i + 1] *
                 -parameter_identify_U.w + parameter_identify_DW.P_past[3 * i] *
                 -parameter_identify_U.iq) + parameter_identify_DW.P_past[3 * i
                + 2] * parameter_identify_U.uq) * rtb_Filter;
    denAccum += xt_tmp_0;
    xt_1 += xt_tmp_0;
    rtb_Filter1 += rtb_Filter * parameter_identify_DW.theta_past[i];
  }

  /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
   *  Inport: '<Root>/iq'
   *  Inport: '<Root>/uq'
   *  Inport: '<Root>/w'
   *  SignalConversion generated from: '<S4>/ SFunction '
   */
  rtb_Filter_j = (xt_tmp_1 + rtb_Filter_j) - rtb_Filter1;
  for (i = 0; i < 3; i++) {
    rtb_Filter1 = xt[i];
    xt_0[3 * i] = -parameter_identify_U.iq * rtb_Filter1;
    xt_0[3 * i + 1] = -parameter_identify_U.w * rtb_Filter1;
    xt_0[3 * i + 2] = parameter_identify_U.uq * rtb_Filter1;
    parameter_identify_DW.theta_past[i] += ((parameter_identify_DW.P_past[i + 3]
      * -parameter_identify_U.w + parameter_identify_DW.P_past[i] *
      -parameter_identify_U.iq) + parameter_identify_DW.P_past[i + 6] *
      parameter_identify_U.uq) / (xt_1 + 1.0) * rtb_Filter_j;
  }

  for (i = 0; i < 3; i++) {
    for (xt_tmp = 0; xt_tmp < 3; xt_tmp++) {
      tmp_1 = 3 * xt_tmp + i;
      tmp[tmp_1] = 0.0;
      tmp[tmp_1] += xt_0[3 * xt_tmp] * parameter_identify_DW.P_past[i];
      tmp[tmp_1] += xt_0[3 * xt_tmp + 1] * parameter_identify_DW.P_past[i + 3];
      tmp[tmp_1] += xt_0[3 * xt_tmp + 2] * parameter_identify_DW.P_past[i + 6];
    }

    for (xt_tmp = 0; xt_tmp < 3; xt_tmp++) {
      tmp_1 = 3 * xt_tmp + i;
      tmp_0[tmp_1] = parameter_identify_DW.P_past[tmp_1] -
        ((parameter_identify_DW.P_past[3 * xt_tmp + 1] * tmp[i + 3] +
          parameter_identify_DW.P_past[3 * xt_tmp] * tmp[i]) +
         parameter_identify_DW.P_past[3 * xt_tmp + 2] * tmp[i + 6]) / (denAccum
        + 1.0);
    }
  }

  memcpy(&parameter_identify_DW.P_past[0], &tmp_0[0], 9U * sizeof(real_T));

  /* Outport: '<Root>/R1' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  parameter_identify_Y.R1 = parameter_identify_DW.theta_past[0] /
    parameter_identify_DW.theta_past[2];

  /* Outport: '<Root>/Ke1' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  parameter_identify_Y.Ke1 = parameter_identify_DW.theta_past[1] /
    parameter_identify_DW.theta_past[2];

  /* Outport: '<Root>/Ld1' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  parameter_identify_Y.Ld1 = 1.0 / parameter_identify_DW.theta_past[2];

  /* Update for UnitDelay: '<S5>/UD'
   *
   * Block description for '<S5>/UD':
   *
   *  Store in Global RAM
   */
  parameter_identify_DW.UD_DSTATE = rtb_TSamp;

  /* Update for UnitDelay: '<S6>/UD' incorporates:
   *  SampleTimeMath: '<S6>/TSamp'
   *
   * About '<S6>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *
   * Block description for '<S6>/UD':
   *
   *  Store in Global RAM
   */
  parameter_identify_DW.UD_DSTATE_c = rtb_TSamp_i_tmp;

  /* Update for UnitDelay: '<S3>/UD'
   *
   * Block description for '<S3>/UD':
   *
   *  Store in Global RAM
   */
  parameter_identify_DW.UD_DSTATE_d = rtb_TSamp_i_tmp;
}

/* Model initialize function */
void parameter_identify_initialize(void)
{
  /* Start for DataStoreMemory: '<S1>/Data Store Memory1' */
  parameter_identify_DW.theta_past[0] = 1.0;
  parameter_identify_DW.theta_past[1] = 1.0;
  parameter_identify_DW.theta_past[2] = 10000.0;

  /* Start for DataStoreMemory: '<S1>/Data Store Memory' */
  memcpy(&parameter_identify_DW.P_past[0], &parameter_identify_ConstP.pooled7[0],
         9U * sizeof(real_T));

  /* Start for DataStoreMemory: '<S2>/Data Store Memory' */
  memcpy(&parameter_identify_DW.P_past_l[0], &parameter_identify_ConstP.pooled7
         [0], 9U * sizeof(real_T));

  /* Start for DataStoreMemory: '<S2>/Data Store Memory1' */
  parameter_identify_DW.theta_past_k[0] = 1.0;
  parameter_identify_DW.theta_past_k[1] = 1.0;
  parameter_identify_DW.theta_past_k[2] = 10000.0;
}

/* Model terminate function */
void parameter_identify_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */



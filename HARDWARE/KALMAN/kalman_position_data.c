/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: kalman_position_data.c
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

/* Constant parameters (default storage) */
const ConstP_kalman_position_T kalman_position_ConstP = {
  /* Expression: p.R{1}
   * Referenced by: '<S1>/R1'
   */
  { 0.1, 0.0, 0.0, 0.1 },

  /* Expression: p.Q
   * Referenced by: '<S1>/Q'
   */
  { 0.1, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.031622776601683791, 0.0,
    0.0, 0.0, 0.0, 0.031622776601683791 },

  /* Expression: p.InitialCovariance
   * Referenced by: '<S1>/DataStoreMemory - P'
   */
  { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    1.0 }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

/*
 * File: PID_POS_data.c
 *
 * Real-Time Workshop code generated for Simulink model PID_POS.
 *
 * Model version                        : 1.1
 * Real-Time Workshop file version      : 7.4  (R2009b)  29-Jun-2009
 * Real-Time Workshop file generated on : Tue Apr 06 20:22:26 2021
 * TLC version                          : 7.4 (Jul 14 2009)
 * C/C++ source code generated on       : Tue Apr 06 20:22:26 2021
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "PID_POS.h"
#include "PID_POS_private.h"

/* Block parameters (auto storage) */
Parameters_PID_POS PID_POS_P = {
  1.0,                                 /* Expression: P
                                        * Referenced by: '<S1>/Proportional Gain'
                                        */
  0.005,                               /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S1>/Integrator'
                                        */
  0.0,                                 /* Expression: InitialConditionForIntegrator
                                        * Referenced by: '<S1>/Integrator'
                                        */
  0.0,                                 /* Expression: D
                                        * Referenced by: '<S1>/Derivative Gain'
                                        */
  0.005,                               /* Computed Parameter: Filter_gainval
                                        * Referenced by: '<S1>/Filter'
                                        */
  0.0,                                 /* Expression: InitialConditionForFilter
                                        * Referenced by: '<S1>/Filter'
                                        */
  100.0,                               /* Expression: N
                                        * Referenced by: '<S1>/Filter Coefficient'
                                        */
  12.0,                                /* Expression: 12
                                        * Referenced by: '<Root>/Saturation'
                                        */
  -12.0,                               /* Expression: -12
                                        * Referenced by: '<Root>/Saturation'
                                        */
  1.0                                  /* Expression: I
                                        * Referenced by: '<S1>/Integral Gain'
                                        */
};

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */

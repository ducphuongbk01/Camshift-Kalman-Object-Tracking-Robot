/*
 * File: PID_POS.c
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

/* Exported block signals */
real_T SetPoint;                       /* '<Root>/SetPoint' */
real_T Encoder;                        /* '<Root>/Encoder' */
real_T udk;                            /* '<Root>/Saturation' */

/* Block states (auto storage) */
D_Work_PID_POS PID_POS_DWork;

/* Real-time model */
RT_MODEL_PID_POS PID_POS_M_;
RT_MODEL_PID_POS *PID_POS_M = &PID_POS_M_;

/* Model step function */
void PID_POS_step(void)
{
  real_T rtb_Sum;
  real_T rtb_Sum_c;
  real_T rtb_FilterCoefficient;

  /* Sum: '<Root>/Sum' incorporates:
   *  Inport: '<Root>/Encoder'
   *  Inport: '<Root>/SetPoint'
   */
  rtb_Sum = SetPoint - Encoder;

  /* Gain: '<S1>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S1>/Filter'
   *  Gain: '<S1>/Derivative Gain'
   *  Sum: '<S1>/SumD'
   */
  rtb_FilterCoefficient = (PID_POS_P.DerivativeGain_Gain * rtb_Sum -
    PID_POS_DWork.Filter_DSTATE) * PID_POS_P.FilterCoefficient_Gain;

  /* Sum: '<S1>/Sum' incorporates:
   *  DiscreteIntegrator: '<S1>/Integrator'
   *  Gain: '<S1>/Proportional Gain'
   */
  rtb_Sum_c = (PID_POS_P.ProportionalGain_Gain * rtb_Sum +
               PID_POS_DWork.Integrator_DSTATE) + rtb_FilterCoefficient;

  /* Saturate: '<Root>/Saturation' */
  udk = rt_SATURATE(rtb_Sum_c, PID_POS_P.Saturation_LowerSat,
                    PID_POS_P.Saturation_UpperSat);

  /* Update for DiscreteIntegrator: '<S1>/Integrator' incorporates:
   *  Gain: '<S1>/Integral Gain'
   */
  PID_POS_DWork.Integrator_DSTATE = PID_POS_P.IntegralGain_Gain * rtb_Sum *
    PID_POS_P.Integrator_gainval + PID_POS_DWork.Integrator_DSTATE;

  /* Update for DiscreteIntegrator: '<S1>/Filter' */
  PID_POS_DWork.Filter_DSTATE = PID_POS_P.Filter_gainval * rtb_FilterCoefficient
    + PID_POS_DWork.Filter_DSTATE;
}

/* Model initialize function */
void PID_POS_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(PID_POS_M, (NULL));

  /* block I/O */

  /* exported global signals */
  udk = 0.0;

  /* states (dwork) */
  (void) memset((void *)&PID_POS_DWork, 0,
                sizeof(D_Work_PID_POS));

  /* external inputs */
  SetPoint = 0.0;
  Encoder = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S1>/Integrator' */
  PID_POS_DWork.Integrator_DSTATE = PID_POS_P.Integrator_IC;

  /* InitializeConditions for DiscreteIntegrator: '<S1>/Filter' */
  PID_POS_DWork.Filter_DSTATE = PID_POS_P.Filter_IC;
}

/* Model terminate function */
void PID_POS_terminate(void)
{
  /* (no terminate code required) */
	udk = 0;
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */

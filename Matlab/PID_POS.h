/*
 * File: PID_POS.h
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

#ifndef RTW_HEADER_PID_POS_h_
#define RTW_HEADER_PID_POS_h_
#ifndef PID_POS_COMMON_INCLUDES_
# define PID_POS_COMMON_INCLUDES_
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#include "rt_SATURATE.h"
#endif                                 /* PID_POS_COMMON_INCLUDES_ */

#include "PID_POS_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S1>/Integrator' */
  real_T Filter_DSTATE;                /* '<S1>/Filter' */
} D_Work_PID_POS;

/* Parameters (auto storage) */
struct Parameters_PID_POS_ {
  real_T ProportionalGain_Gain;        /* Expression: P
                                        * Referenced by: '<S1>/Proportional Gain'
                                        */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S1>/Integrator'
                                        */
  real_T Integrator_IC;                /* Expression: InitialConditionForIntegrator
                                        * Referenced by: '<S1>/Integrator'
                                        */
  real_T DerivativeGain_Gain;          /* Expression: D
                                        * Referenced by: '<S1>/Derivative Gain'
                                        */
  real_T Filter_gainval;               /* Computed Parameter: Filter_gainval
                                        * Referenced by: '<S1>/Filter'
                                        */
  real_T Filter_IC;                    /* Expression: InitialConditionForFilter
                                        * Referenced by: '<S1>/Filter'
                                        */
  real_T FilterCoefficient_Gain;       /* Expression: N
                                        * Referenced by: '<S1>/Filter Coefficient'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 12
                                        * Referenced by: '<Root>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: -12
                                        * Referenced by: '<Root>/Saturation'
                                        */
  real_T IntegralGain_Gain;            /* Expression: I
                                        * Referenced by: '<S1>/Integral Gain'
                                        */
};

/* Real-time Model Data Structure */
struct RT_MODEL_PID_POS {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern Parameters_PID_POS PID_POS_P;

/* Block states (auto storage) */
extern D_Work_PID_POS PID_POS_DWork;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  RTW declares the memory for these signals
 * and exports their symbols.
 *
 */
extern real_T SetPoint;                /* '<Root>/SetPoint' */
extern real_T Encoder;                 /* '<Root>/Encoder' */
extern real_T udk;                     /* '<Root>/Saturation' */

/* Model entry point functions */
extern void PID_POS_initialize(void);
extern void PID_POS_step(void);
extern void PID_POS_terminate(void);

/* Real-time Model object */
extern RT_MODEL_PID_POS *PID_POS_M;

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
 * '<Root>' : PID_POS
 * '<S1>'   : PID_POS/Discrete PID Controller
 */
#endif                                 /* RTW_HEADER_PID_POS_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */

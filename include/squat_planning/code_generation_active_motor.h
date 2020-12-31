//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: code_generation_active_motor.h
//
// Code generated for Simulink model 'code_generation_active_motor'.
//
// Model version                  : 1.4
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Fri Aug 28 16:18:11 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_code_generation_active_motor_h_
#define RTW_HEADER_code_generation_active_motor_h_
#include <stddef.h>
#include <cmath>
#include <math.h>
#ifndef code_generation_active_motor_COMMON_INCLUDES_
# define code_generation_active_motor_COMMON_INCLUDES_
#include "squat_planning/rtwtypes.h"
#endif                                 // code_generation_active_motor_COMMON_INCLUDES_ 
// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Forward declaration for rtModel
typedef struct tag_RTM RT_MODEL;

// External inputs (root inport signals with default storage)
typedef struct {
  real_T Y_d;                          // '<Root>/Y_d'
  real_T t_sim0;                       // '<Root>/t_sim0'
  real_T t_sim;                        // '<Root>/t_sim'
  real_T X_d;                          // '<Root>/X_d'
  real_T forward_vel;                  // '<Root>/forward_vel'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real_T ankle;                        // '<Root>/ankle'
  real_T knee;                         // '<Root>/knee'
  real_T hip;                          // '<Root>/hip'
  real_T wheel;                        // '<Root>/wheel'
} ExtY;

// Real-time Model Data Structure
struct tag_RTM {
  const char_T * volatile errorStatus;
};

// Class declaration for model code_generation_active_motor
class code_generation_active_motorModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  code_generation_active_motorModelClass();

  // Destructor
  ~code_generation_active_motorModelClass();

  // Real-Time Model get method
  RT_MODEL * getRTM();

  // private data and function members
 private:
  // Real-Time Model
  RT_MODEL rtM;

  // private member function(s) for subsystem '<Root>'
  real_T norm(const real_T x[2]);
  void mldivide(const real_T A[4], const real_T B_0[2], real_T Y[2]);
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'code_generation_active_motor'
//  '<S1>'   : 'code_generation_active_motor/desire_joint_angle'
//  '<S2>'   : 'code_generation_active_motor/desire_joint_angle/Cartesian_Space_programme'
//  '<S3>'   : 'code_generation_active_motor/desire_joint_angle/desire_motor_angle'
//  '<S4>'   : 'code_generation_active_motor/desire_joint_angle/desire_passive_angle'

#endif                                 // RTW_HEADER_code_generation_active_motor_h_ 

//
// File trailer for generated code.
//
// [EOF]
//

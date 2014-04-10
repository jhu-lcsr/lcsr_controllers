
#include "joint_friction_compensator_hss.h"

using namespace lcsr_controllers;

bool JointFrictionCompensatorHSS::Valid(
    double static_effort_low, 
    double static_effort_high,
    double deadband_low, 
    double deadband_high,
    double p_gain)
{
  return
    p_gain >= 0.0 &&
    static_effort_low <= 0.0 &&
    static_effort_high >= 0.0 &&
    deadband_low <= 0.0 &&
    deadband_high >= 0.0 &&
    deadband_low >= static_effort_low / p_gain &&
    deadband_high <= static_effort_high / p_gain;
}

double JointFrictionCompensatorHSS::Compensate(
    double static_effort, 
    double deadband, 
    double p_gain,
    double joint_position_error,
    double joint_velocity,
    double eps)
{
  return Compensate(
    -static_effort, 
    static_effort,
    -deadband, 
    deadband,
    p_gain,
    joint_position_error,
    joint_velocity,
    eps);
}

double JointFrictionCompensatorHSS::Compensate(
    double static_effort_low, 
    double static_effort_high,
    double deadband_low, 
    double deadband_high,
    double p_gain,
    double joint_position_error,
    double joint_velocity,
    double eps)
{
  double &q_err = joint_position_error;

  double q_L = static_effort_low / p_gain;
  double q_H = static_effort_high / p_gain;

  double &d_L = deadband_low;
  double &d_H = deadband_high;

  if(q_L < q_err && q_err < q_H) {
    if(joint_velocity > eps) {
      if(q_err > 0) {
        return static_effort_high;
      } else if(q_err > d_L) {
        return 0.0;
      } else {
        return static_effort_low;
      }
    } else if(joint_velocity < -eps) {
      if(q_err > d_H) {
        return static_effort_high;
      } else if(q_err > 0.0) {
        return 0.0;
      } else {
        return static_effort_low;
      }
    } else {
      if(q_err > d_H) {
        return static_effort_high;
      } else if(q_err > d_L) {
        return 0.0;
      } else {
        return static_effort_low;
      }
    }
  }

  return p_gain * q_err;
}


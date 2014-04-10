
#include "joint_friction_compensator_hss.h"

using namespace lcsr_controllers;

JointFrictionCompensatorHSS::JointFrictionCompensatorHSS(
    double p_gain_,
    double static_effort_low_, 
    double static_effort_high_,
    double deadband_low_, 
    double deadband_high_) :
  p_gain(p_gain_),
  static_effort_low(static_effort_low_), 
  static_effort_high(static_effort_high_),
  deadband_low(deadband_low_), 
  deadband_high(deadband_high_)
{
}


JointFrictionCompensatorHSS::JointFrictionCompensatorHSS(
    double p_gain_,
    double static_effort_,
    double deadband_) :
  p_gain(p_gain_),
  static_effort_low(-static_effort_), 
  static_effort_high(static_effort_),
  deadband_low(-deadband_), 
  deadband_high(deadband_)
{

}

bool JointFrictionCompensatorHSS::valid() const
{
  return
    p_gain > 0.0 &&
    static_effort_low < 0.0 &&
    static_effort_high > 0.0 &&
    deadband_low < 0.0 &&
    deadband_high > 0.0 &&
    deadband_low > static_effort_low / p_gain &&
    deadband_high < static_effort_high / p_gain;
}

double JointFrictionCompensatorHSS::compensate(
    const double joint_position_desired,
    const double joint_position,
    const double joint_velocity) const
{
  const double q_err = joint_position_desired - joint_position;
  const double q_nerr = -1.0 * q_err;

  const double q_L = static_effort_low / p_gain;
  const double q_H = static_effort_high / p_gain;

  const double &d_L = deadband_low;
  const double &d_H = deadband_high;

  if(q_L < q_nerr && q_nerr < q_H) {
    if(joint_velocity > 0.0) {
      if(q_nerr > 0) {
        return static_effort_high;
      } else if(q_nerr > d_L) {
        return 0.0;
      } else {
        return static_effort_low;
      }
    } else if(joint_velocity < 0.0) {
      if(q_nerr > d_H) {
        return static_effort_high;
      } else if(q_nerr > 0.0) {
        return 0.0;
      } else {
        return static_effort_low;
      }
    } else {
      if(q_nerr > d_H) {
        return static_effort_high;
      } else if(q_nerr > d_L) {
        return 0.0;
      } else {
        return static_effort_low;
      }
    }
  }

  return p_gain * q_err;
}


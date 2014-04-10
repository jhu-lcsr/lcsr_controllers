#ifndef __LCSR_CONTROLLERS_JOINT_FRICTION_COMPENSATOR_HSS_H
#define __LCSR_CONTROLLERS_JOINT_FRICTION_COMPENSATOR_HSS_H

namespace lcsr_controllers {

  class JointFrictionCompensatorHSS {
  public:
    static bool Valid(
        double static_effort_low, 
        double static_effort_high,
        double deadband_low, 
        double deadband_high,
        double p_gain);

    static double Compensate(
        double static_effort_low_, 
        double static_effort_high_,
        double deadband_low_, 
        double deadband_high_,
        double p_gain,
        double joint_position_error,
        double joint_velocity,
        double eps = 0.0);

    static double Compensate(
        double static_effort,
        double deadband,
        double p_gain,
        double joint_position_error,
        double joint_velocity,
        double eps = 0.0);
  };
}

#endif // ifndef __LCSR_CONTROLLERS_JOINT_FRICTION_COMPENSATOR_HSS_H

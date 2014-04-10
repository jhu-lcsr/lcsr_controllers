#ifndef __LCSR_CONTROLLERS_JOINT_FRICTION_COMPENSATOR_HSS_H
#define __LCSR_CONTROLLERS_JOINT_FRICTION_COMPENSATOR_HSS_H

namespace lcsr_controllers {

  class JointFrictionCompensatorHSS {
  public:
    JointFrictionCompensatorHSS(
        double p_gain_,
        double static_effort_low_, 
        double static_effort_high_,
        double deadband_low_, 
        double deadband_hig_h);

    JointFrictionCompensatorHSS(
        double p_gain_,
        double static_effort_,
        double deadband_);

    bool valid() const;

    double compensate(
        const double joint_position_desired,
        const double joint_position,
        const double joint_velocity) const;

    double 
      p_gain,
      static_effort_low, 
      static_effort_high,
      deadband_low, 
      deadband_high;
  };
}

#endif // ifndef __LCSR_CONTROLLERS_JOINT_FRICTION_COMPENSATOR_HSS_H

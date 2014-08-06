#ifndef __LCSR_CONTROLLERS_CARTESIAN_LOGISTIC_SERVO_H
#define __LCSR_CONTROLLERS_CARTESIAN_LOGISTIC_SERVO_H

#include <iostream>

#include <boost/shared_ptr.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>

//#include <rtt_ros_tools/throttles.h>
#include <tf/tf.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#include <visualization_msgs/Marker.h>

namespace lcsr_controllers {
  class CartesianLogisticServo : public RTT::TaskContext
  {
    // RTT Properties
    std::string robot_description_;
    std::string robot_description_param_;
    std::string root_link_;
    std::string tip_link_;
    std::string target_frame_;
    double max_linear_rate_;
    double max_angular_rate_;

    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> positions_in_port_;
    RTT::OutputPort<KDL::FrameVel> framevel_out_port_;

    // RTT Debug Ports
    RTT::OperationCaller<geometry_msgs::TransformStamped(const std::string&,
      const std::string&)> tf_lookup_transform_;
    RTT::OperationCaller<geometry_msgs::TransformStamped(const geometry_msgs::TransformStamped&)> 
      tf_broadcast_transform_;

  public:
    CartesianLogisticServo(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  private:

    // Kinematic properties
    unsigned int n_dof_;
    KDL::Chain kdl_chain_;
    KDL::Tree kdl_tree_;

    // Working variables
    KDL::JntArrayVel positions_;

    // Joint limits
    KDL::JntArray joint_limits_min_;
    KDL::JntArray joint_limits_max_;

    // KDL FK solver
    boost::shared_ptr<KDL::ChainFkSolverVel> fk_solver_vel_;

    // KDL Jacobian
    boost::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;

    geometry_msgs::TransformStamped target_frame_limited_msg_;
    geometry_msgs::TransformStamped tip_frame_msg_;
    KDL::Frame tip_frame_des_;
    KDL::Twist tip_frame_twist_;
    KDL::Frame frame_limited_;
    KDL::FrameVel framevel_limited_;

    rtt_ros_tools::PeriodicThrottle ros_publish_throttle_;

    ros::Time update_time_;
    ros::Time last_update_time_;

    bool warn_flag_;

  };
}


#endif // ifndef __LCSR_CONTROLLERS_CARTESIAN_LOGISTIC_SERVO_H

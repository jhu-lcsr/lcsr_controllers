
#ifndef __LCSR_CONTROLLERS_JOINT_TRAJ_GENERATOR_KDL_H
#define __LCSR_CONTROLLERS_JOINT_TRAJ_GENERATOR_KDL_H

#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <rtt/RTT.hpp>
#include <rtt/Port.hpp>

#include <kdl/jntarrayvel.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>

#include <rtt_ros_tools/throttles.h>

#include <conman/hook.h>

#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>

namespace lcsr_controllers {
  class JointTrajGeneratorRML : public RTT::TaskContext
  {
    // RTT Properties
    std::string robot_description_;
    std::string root_link_;
    std::string tip_link_;
    float velocity_smoothing_factor_;
    Eigen::VectorXd 
      max_velocities_,
      max_accelerations_,
      max_jerks_;

    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> joint_velocity_in_;
    RTT::InputPort<Eigen::VectorXd> joint_position_cmd_in_;

    RTT::OutputPort<Eigen::VectorXd> joint_position_out_;
    RTT::OutputPort<Eigen::VectorXd> joint_velocity_out_;

    RTT::InputPort<trajectory_msgs::JointTrajectoryPoint> joint_position_cmd_ros_in_;
    RTT::InputPort<trajectory_msgs::JointTrajectory> joint_traj_cmd_in_;
    RTT::OutputPort<sensor_msgs::JointState> joint_state_desired_out_;

  public:
    JointTrajGeneratorRML(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();

  private:

    // Robot model
    unsigned int n_dof_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;

    std::vector<KDL::VelocityProfile_Trap> trajectories_;
    std::vector<RTT::Seconds> trajectory_start_times_;
    std::vector<RTT::Seconds> trajectory_end_times_;

    // State
    Eigen::VectorXd
      position_tolerance_,
      joint_position_,
      joint_position_last_,
      joint_position_cmd_,
      joint_position_sample_,
      joint_velocity_,
      joint_velocity_raw_,
      joint_velocity_sample_;

    trajectory_msgs::JointTrajectoryPoint joint_position_cmd_ros_;
    trajectory_msgs::JointTrajectory joint_traj_cmd_;
    sensor_msgs::JointState joint_state_desired_;
    rtt_ros_tools::PeriodicThrottle ros_publish_throttle_;

    bool has_last_position_data_;

    // Conman interface
    boost::shared_ptr<conman::Hook> conman_hook_;

    //////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////

  protected:
    size_t n_joints_;
    std::vector<std::string> joint_names_;
    std::vector<double> position_tolerances_;
    std::vector<double> commanded_efforts_;
    std::vector<boost::shared_ptr<const urdf::Joint> > urdf_joints_;

    size_t point_index_;
    ros::Time commanded_start_time_;

  private:
    ros::NodeHandle nh_;
    int loop_count_;
    int decimation_;

    //! Output information about the current RML
    void rml_debug(const RTT::LoggerLevel level);

    //! Trajectory Generator
    boost::shared_ptr<ReflexxesAPI> rml_;
    boost::shared_ptr<RMLPositionInputParameters> rml_in_;
    boost::shared_ptr<RMLPositionOutputParameters> rml_out_;
    RMLPositionFlags rml_flags_;
    ros::Time traj_start_time_;

    //! Trajectory parameters
    double sampling_resolution_;
    bool new_reference_;
    bool recompute_trajectory_;

    // Command subscriber
    ros::Subscriber trajectory_command_sub_;
    void trajectoryCommandCB(const trajectory_msgs::JointTrajectoryConstPtr& msg);
    void setTrajectoryCommand(const trajectory_msgs::JointTrajectoryConstPtr& msg);


    //! A via point structure for trajectory points
    struct ViaPoint {
      ViaPoint(size_t n_dof, ros::Time start_time_ = ros::Time(0.0), ros::Time end_time_ = ros::Time(0.0)) :
        positions(n_dof, 0.0),
        velocities(n_dof,0.0),
        accelerations(n_dof, 0.0),
        start_time(start_time_),
        end_time(end_time_) 
      { }

      Eigen::VectorXd positions;
      Eigen::VectorXd velocities;
      Eigen::VectorXd accelerations;
      ros::Time start_time;
      ros::Time end_time;

      //! End-Time comparison function for binary search
      static bool StartTimeCompare(const ViaPoint &v1, const ViaPoint &v2) { 
        return v1.start_time < v2.start_time;
      }

      //! End-Time comparison function for binary search
      static bool EndTimeCompare(const ViaPoint &v1, const ViaPoint &v2) { 
        return v1.end_time < v2.end_time;
      }
    };

    typedef std::list<ViaPoint> Vias;

    //! Via points to follow
    Vias vias_;

    bool new_via_goal_;
  };
}


#endif // ifndef __LCSR_CONTROLLERS_JOINT_TRAJ_GENERATOR_KDL_H

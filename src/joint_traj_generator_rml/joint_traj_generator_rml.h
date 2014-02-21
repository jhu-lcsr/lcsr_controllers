
#ifndef __LCSR_CONTROLLERS_JOINT_TRAJ_GENERATOR_RML_H
#define __LCSR_CONTROLLERS_JOINT_TRAJ_GENERATOR_RML_H

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
  public:
    // RTT Properties
    std::string robot_description_;
    std::string root_link_;
    std::string tip_link_;
    unsigned int n_dof_;
    double sampling_resolution_;
    Eigen::VectorXd 
      position_tolerance_,
      max_velocities_,
      max_accelerations_,
      max_jerks_;

  protected:
    // RTT Ports
    RTT::InputPort<Eigen::VectorXd> joint_position_in_;
    RTT::InputPort<Eigen::VectorXd> joint_velocity_in_;
    RTT::InputPort<Eigen::VectorXd> joint_position_cmd_in_;

    RTT::OutputPort<Eigen::VectorXd> joint_position_out_;
    RTT::OutputPort<Eigen::VectorXd> joint_velocity_out_;

    RTT::InputPort<trajectory_msgs::JointTrajectoryPoint> joint_traj_point_cmd_in_;
    RTT::InputPort<trajectory_msgs::JointTrajectory> joint_traj_cmd_in_;
    RTT::OutputPort<sensor_msgs::JointState> joint_state_desired_out_;

  public:
    JointTrajGeneratorRML(std::string const& name);
    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();
    
    //! A trajectory segment structure for internal use 
    // This structure is used so that trajectory points can be decoupled from
    // each-other. The standard ROS trajectory message includes a timestamp
    // designating the start time, and then each point in the trajectory is
    // stamped relative to that one. Since this controller may splice different
    // trajectories together, re first translate them into an absolute
    // representation, where each point has a well-defined start and end time.
    struct TrajSegment 
    {
      TrajSegment(
          size_t n_dof, 
          bool flexible_ = false) :
        active(false),
        flexible(flexible_),
        start_time(ros::Time(0,0)),
        goal_time(ros::Time(0,0)), 
        expected_time(ros::Time(0,0)), 
        goal_positions(Eigen::VectorXd::Constant(n_dof,0.0)),
        goal_velocities(Eigen::VectorXd::Constant(n_dof,0.0)),
        goal_accelerations(Eigen::VectorXd::Constant(n_dof,0.0))
      { }

      bool active;
      bool flexible;
      ros::Time start_time;
      ros::Time goal_time;
      ros::Time expected_time;
      Eigen::VectorXd goal_positions;
      Eigen::VectorXd goal_velocities;
      Eigen::VectorXd goal_accelerations;

      //! End-Time comparison function for binary search
      static bool StartTimeCompare(const TrajSegment &s1, const TrajSegment &s2) { 
        return s1.start_time < s2.start_time;
      }

      //! End-Time comparison function for binary search
      static bool GoalTimeCompare(const TrajSegment &s1, const TrajSegment &s2) { 
        return s1.goal_time < s2.goal_time;
      }
    };
    
    //! A container of trajectory segments with low complexity front and back modification
    typedef std::list<TrajSegment> TrajSegments;

    //! Segments to follow
    TrajSegments segments_;

    //! Convert a ROS trajectory message to a list of TrajSegments
    static bool TrajectoryMsgToSegments(
        const trajectory_msgs::JointTrajectory &msg,
        const size_t n_dof,
        const ros::Time trajectory_start_time,
        TrajSegments &segments);

    //! Update the one trajectory with points from another
    static bool UpdateTrajectory(
        TrajSegments &current_segments,
        const TrajSegments &new_segments);

    //! Configure some RML structures from this tasks's properties
    bool configureRML(
        boost::shared_ptr<ReflexxesAPI> &rml,
        boost::shared_ptr<RMLPositionInputParameters> &rml_in,
        boost::shared_ptr<RMLPositionOutputParameters> &rml_out,
        RMLPositionFlags &rml_flags) const;
    
    /** \brief Sample the trajectory based on the current set of segments and robot state
     * This function does not change the state of the component, so it can be
     * used easily in testing or with lookaheads.
     */
    bool sampleTrajectory(
        const ros::Time rtt_now,
        const bool force_recompute_trajectory,
        const Eigen::VectorXd &joint_position,
        const Eigen::VectorXd &joint_velocity,
        boost::shared_ptr<ReflexxesAPI> rml,
        boost::shared_ptr<RMLPositionInputParameters> rml_in,
        boost::shared_ptr<RMLPositionOutputParameters> rml_out,
        RMLPositionFlags &rml_flags,
        JointTrajGeneratorRML::TrajSegments &segments,
        Eigen::VectorXd &joint_position_sample,
        Eigen::VectorXd &joint_velocity_sample) const;

    //! Output information about some RML input parameters
    static void RMLLog(
        const RTT::LoggerLevel level,
        const boost::shared_ptr<RMLPositionInputParameters> rml_in);

  protected:

    //! Trajectory Generator
    boost::shared_ptr<ReflexxesAPI> rml_;
    boost::shared_ptr<RMLPositionInputParameters> rml_in_;
    boost::shared_ptr<RMLPositionOutputParameters> rml_out_;
    RMLPositionFlags rml_flags_;

    // Robot model
    std::vector<std::string> joint_names_;
    std::map<std::string,size_t> joint_name_index_map_;

    // State
    Eigen::VectorXd
      joint_position_,
      joint_position_cmd_,
      joint_position_sample_,
      joint_velocity_,
      joint_velocity_sample_;

    trajectory_msgs::JointTrajectoryPoint joint_traj_point_cmd_;
    trajectory_msgs::JointTrajectory joint_traj_cmd_;
    sensor_msgs::JointState joint_state_desired_;
    rtt_ros_tools::PeriodicThrottle ros_publish_throttle_;

    // Conman interface
    boost::shared_ptr<conman::Hook> conman_hook_;

  };
}


#endif // ifndef __LCSR_CONTROLLERS_JOINT_TRAJ_GENERATOR_RML_H

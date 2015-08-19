
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <rtt_rosparam/rosparam.h>

#include <rtt_ros_tools/tools.h>
#include <rtt_roscomm/rtt_rostopic.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <kdl_urdf_tools/tools.h>

#include <lcsr_controllers/inertia_utils.h>

#include "id_controller_kdl.h"

using namespace lcsr_controllers;

IDControllerKDL::IDControllerKDL(std::string const& name) :
  TaskContext(name)
  // Properties
  ,robot_description_param_("/robot_description")
  ,robot_description_("")
  ,root_link_("")
  ,tip_link_("")
  ,gravity_(3)
  ,compensate_gravity_(true)
  ,compensate_coriolis_(true)
  ,compensate_inertial_(false)
  ,compensate_end_effector_(true)
  // Working variables
  ,n_dof_(0)
  ,kdl_tree_()
  ,kdl_chain_()
  ,id_solver_(NULL)
  ,ext_wrenches_()
  ,positions_()
  ,accelerations_()
  ,torques_()
  // Inertia
  ,inertia_map_()
  ,inertia_mass_rate_(0.0)
  ,inertia_com_rate_(0.0)
  ,max_attached_mass_(0.0)
  // Throttles
  ,debug_throttle_(0.05)
  ,last_update_time_()
{
  // Zero gravity
  gravity_.setZero();
    
  // Declare properties
  this->addProperty("robot_description_param",robot_description_param_)
    .doc("The ROS parameter for the URDF xml string. (default: '/robot_description')");
  this->addProperty("robot_description",robot_description_)
    .doc("The WAM URDF xml string.");
  // TODO: Get gravity from world frame
  this->addProperty("gravity",gravity_)
    .doc("The gravity vector in the root link frame.");
  this->addProperty("root_link",root_link_)
    .doc("The root link for the controller.");
  this->addProperty("tip_link",tip_link_)
    .doc("The tip link for the controller.");
  this->addProperty("compensate_gravity",compensate_gravity_)
    .doc("Will apply a force countering forces due to gravity if true.");
  this->addProperty("compensate_coriolis",compensate_coriolis_)
    .doc("Will apply a force countering coriolis forces if true.");
  this->addProperty("compensate_inertial",compensate_inertial_)
    .doc("Will apply a force countering inertial forces due to acceleration if true.");
  this->addProperty("compensate_end_effector",compensate_end_effector_)
    .doc("Will compute a wrench on the tip link if true.");
  this->addProperty("inertia_mass_rate",inertia_mass_rate_)
    .doc("The maximum rate in kg/s at which mass can be added and removed from the end-effector.");
  this->addProperty("inertia_com_rate",inertia_com_rate_)
    .doc("The maximum rate in m/s at which the center of mass of the attached inertia can be moved.");
  this->addProperty("max_attached_mass",max_attached_mass_)
    .doc("The maximum mass that can be attached to the arm.");

  // Configure data ports
  this->ports()->addPort("joint_position_in", joint_position_in_);
  this->ports()->addPort("joint_velocity_in", joint_velocity_in_);
  this->ports()->addPort("end_effector_masses_in", end_effector_masses_in_)
    .doc("Additional masses rigidly attached to the end-effector. Given as (x,y,z,m) in the coordinate frame given by the tip_link property of this component");
  this->ports()->addPort("joint_effort_out", joint_effort_out_)
    .doc("Output port: nx1 vector of joint torques. (n joints)");

  // Add the port and stream it to a ROS topic
  this->ports()->addPort("ext_wrenches_debug_out", ext_wrenches_debug_out_);
  ext_wrenches_debug_out_.createStream(rtt_roscomm::topic("~/"+this->getName()+"/wrenches"));

  this->ports()->addPort("cogs_debug_out", cogs_debug_out_);
  cogs_debug_out_.createStream(rtt_roscomm::topicBuffer("~/"+this->getName()+"/cogs",32));

  this->ports()->addPort("end_effector_inertias_in", end_effector_inertias_in_);
  end_effector_inertias_in_.createStream(rtt_roscomm::topic("~/"+this->getName()+"/end_effector_inertias"));
}

bool IDControllerKDL::configureHook()
{
  // ROS parameters
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this->getProvider<rtt_rosparam::ROSParam>("rosparam");
  // Get absoluate parameters
  rosparam->getComponentPrivate("robot_description_param");
  rosparam->getParam(robot_description_param_, "robot_description");
  // Get private parameters
  rosparam->getComponentPrivate("root_link");
  rosparam->getComponentPrivate("tip_link");
  rosparam->getComponentPrivate("gravity");
  rosparam->getComponentPrivate("compensate_gravity");
  rosparam->getComponentPrivate("compensate_coriolis");
  rosparam->getComponentPrivate("compensate_inertial");
  rosparam->getComponentPrivate("compensate_end_effector");
  rosparam->getComponentPrivate("inertia_mass_rate");
  rosparam->getComponentPrivate("inertia_com_rate");
  rosparam->getComponentPrivate("max_attached_mass");

  RTT::log(RTT::Debug) << "Initializing kinematic and dynamic parameters from \"" << root_link_ << "\" to \"" << tip_link_ <<"\"" << RTT::endlog();

  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  urdf::Model urdf_model;
  if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model))
  {
    RTT::log(RTT::Error) << "Could not initialize robot kinematics!" << RTT::endlog();
    return false;
  }

  // Create a PoseStamped message for each center of gravity
  for(std::vector<KDL::Segment>::const_iterator it=kdl_chain_.segments.begin();
      it != kdl_chain_.segments.end();
      ++it)
  {
    visualization_msgs::Marker cog_pose;
    cog_pose.header.frame_id = it->getName();
    cog_pose.ns = it->getName();
    cog_pose.type = visualization_msgs::Marker::SPHERE;
    cog_pose.frame_locked = true;
    cog_pose.scale.x = 0.02;
    cog_pose.scale.y = 0.02;
    cog_pose.scale.z = 0.02;
    cog_pose.color.r = 28.0/255.0;
    cog_pose.color.g = 217.0/255.0;
    cog_pose.color.b = 214.0/255.0;
    cog_pose.color.a = 128.0;
    cog_pose.pose.position.x = it->getInertia().getCOG().x();
    cog_pose.pose.position.y = it->getInertia().getCOG().y();
    cog_pose.pose.position.z = it->getInertia().getCOG().z();

    cogs_msgs_.push_back(cog_pose);
  }

  // Add ee cog marker
  visualization_msgs::Marker cog_pose;
  cog_pose.header.frame_id = tip_link_;
  cog_pose.ns = "attached";
  cog_pose.type = visualization_msgs::Marker::SPHERE;
  cog_pose.frame_locked = true;
  cog_pose.scale.x = 0.0;
  cog_pose.scale.y = 0.0;
  cog_pose.scale.z = 0.0;
  cog_pose.color.r = 255.0/255.0;
  cog_pose.color.g = 102.0/255.0;
  cog_pose.color.b = 102.0/255.0;
  cog_pose.color.a = 128.0;
  cog_pose.pose.position.x = 0.0;
  cog_pose.pose.position.y = 0.0;
  cog_pose.pose.position.z = 0.0;

  cogs_msgs_.push_back(cog_pose);

  // Create inverse dynamics chainsolver
  id_solver_.reset(
      new KDL::ChainIdSolver_RNE(
        kdl_chain_,
        (compensate_gravity_ ? 1.0 : 0.0) * KDL::Vector(gravity_[0],gravity_[1],gravity_[2])));

  // Create the forward kinematics solver
  fk_solver_.reset( new KDL::ChainFkSolverPos_recursive( kdl_chain_ ) );

  // Resize IO vectors
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_acceleration_.resize(n_dof_);
  joint_effort_.resize(n_dof_);

  // Resize working vectors
  positions_.resize(n_dof_);
  velocities_.resize(n_dof_);
  accelerations_.resize(n_dof_);
  ext_wrenches_.resize(kdl_chain_.getNrOfSegments());
  torques_.resize(n_dof_);

  // Initialize the wrenches
  for(size_t i=0; i<kdl_chain_.getNrOfSegments(); i++) {
    ext_wrenches_[i] = KDL::Wrench::Zero();
  }

  // Prepare ports for realtime processing
  joint_effort_out_.setDataSample(joint_effort_);

  return true;
}

bool IDControllerKDL::startHook()
{
  // Zero accelerations
  joint_acceleration_.setZero();
  // Zero out torque data
  torques_.data.setZero();

  // Initialize ee inertia
  ee_inertia_ = KDL::RigidBodyInertia::Zero();
  total_attached_inertia_ = KDL::RigidBodyInertia::Zero();

  // Initialize last update time
  last_update_time_ = rtt_rosclock::rtt_now();
  return true;
}

void IDControllerKDL::updateHook()
{
  // Get current time
  ros::Time now = rtt_rosclock::rtt_now();
  ros::Duration period = now - last_update_time_;
  last_update_time_ = now;

  // Read in the current joint positions & velocities
  bool new_pos_data = joint_position_in_.readNewest( joint_position_ ) == RTT::NewData;
  bool new_vel_data = joint_velocity_in_.readNewest( joint_velocity_ ) == RTT::NewData;

  KDL::RigidBodyInertia ee_mass;
  bool new_ee_mass = false;

  if(end_effector_masses_in_.readNewest( end_effector_mass_ ) == RTT::NewData) 
  {
    // Make sure the new mass is well-posed
    if(end_effector_mass_.size() == 4 && end_effector_mass_[3] > 1E-4) 
    {
      // Update EE inertia
      ee_mass = KDL::RigidBodyInertia(
          end_effector_mass_[3],
          KDL::Vector(end_effector_mass_[0],
                      end_effector_mass_[1],
                      end_effector_mass_[2]));

      if(!inertia_map_.set_attached_inertia(ee_mass, "__ee__", 0)) {
        RTT::log(RTT::Error) << "End-effector inertia is not well-posed." << RTT::endlog();
      }

      new_ee_mass = true;
    }
  }

  while(end_effector_inertias_in_.read( end_effector_inertia_ ) == RTT::NewData) 
  {
    // Update inertia map
    // Make sure the new mass is well-posed
    if(inertia_map_.update(end_effector_inertia_)) {
      switch(end_effector_inertia_.action) {
        case telemanip_msgs::AttachedInertia::UPDATE:
          RTT::log(RTT::Debug) <<"Updated EE inertia: "<<end_effector_inertia_.ns<< " #" <<end_effector_inertia_.id<<" @ "<<end_effector_inertia_.inertia.m<<" kg"<<RTT::endlog();
          break;
        case telemanip_msgs::AttachedInertia::DELETE:
          RTT::log(RTT::Debug) <<"Deleted EE inertia: "<<end_effector_inertia_.ns<< " #" <<end_effector_inertia_.id<<RTT::endlog();
          break;
      };
    } else {
      switch(end_effector_inertia_.action) {
        case telemanip_msgs::AttachedInertia::UPDATE:
          RTT::log(RTT::Debug) <<"Not updating attached inertia "
            <<end_effector_inertia_.ns <<" #" <<end_effector_inertia_.id
            <<" because it is not well-posed: "
            <<end_effector_inertia_.inertia.m <<RTT::endlog();
          break;
        case telemanip_msgs::AttachedInertia::DELETE:
          RTT::log(RTT::Warning) <<"Not removing inertia "
            <<end_effector_inertia_.ns <<" #" <<end_effector_inertia_.id
            <<" because it is not currently attached."<<RTT::endlog();
          break;
        default:
          RTT::log(RTT::Error) << "Unknown attached inertia action: "<<end_effector_inertia_.action<<RTT::endlog();
          break;
      };
    }
  }

  // Attached cogs
  KDL::RigidBodyInertia inertia_sum;
  bool load_ok = inertia_map_.sum(inertia_sum, max_attached_mass_);

  // Marker message
  visualization_msgs::Marker &attached_inertia = cogs_msgs_[cogs_msgs_.size()-1];
  attached_inertia.scale.x = inertia_sum.getMass() / 100.0;
  attached_inertia.scale.z = inertia_sum.getMass() / 100.0;
  attached_inertia.scale.y = inertia_sum.getMass() / 100.0;
  attached_inertia.pose.position.x = inertia_sum.getCOG()[0];
  attached_inertia.pose.position.y = inertia_sum.getCOG()[1];
  attached_inertia.pose.position.z = inertia_sum.getCOG()[2];

  // Interpolate total attached inertia
  interpolate_inertia(
      total_attached_inertia_,
      total_attached_inertia_,
      inertia_sum,
      inertia_mass_rate_,
      inertia_com_rate_,
      period);

  if(check_inertia(total_attached_inertia_)) {
    ee_inertia_ = total_attached_inertia_;
  }

  if(new_pos_data && new_vel_data) {
    // Get JntArray structures from pos/vel
    positions_.data = joint_position_;
    velocities_.data = joint_velocity_;

    // Compute wwrenches on the end-effector
    if(compensate_end_effector_) {
      // Compute the tip frame from the current joint position
      KDL::Frame tip_frame;
      int ret = fk_solver_->JntToCart(positions_, tip_frame);
      // Compute gravity vector in the tip frame
      KDL::Vector tip_gravity = tip_frame.M.Inverse() * KDL::Vector(gravity_[0], gravity_[1], gravity_[2]);
      KDL::Twist tip_gravity_twist(tip_gravity, KDL::Vector::Zero()); //TODO: Add centripetal acceleration (v*r^2)
      ee_wrench_ = ee_inertia_ * tip_gravity_twist;
      // Compute the external wrench on the tip link
      // Set the wrench (in root_link_ coordinates)
      ext_wrenches_.back() = ee_wrench_;
    } else { 
      // Zero the last wrench
      ext_wrenches_.back() = KDL::Wrench::Zero();
    }

    // Zero out vectors based on configuration
    if(!compensate_coriolis_) {
      KDL::SetToZero(velocities_);
    }
    if(!compensate_inertial_) {
      KDL::SetToZero(accelerations_);
    }

    // Compute inverse dynamics
    // This computes the torques on each joint of the arm as a function of
    // the arm's joint-space position, velocities, accelerations, external
    // forces/torques and gravity.
    if(id_solver_->CartToJnt(
          positions_,
          velocities_,
          accelerations_,
          ext_wrenches_,
          torques_) != 0)
    {
      RTT::log(RTT::Error) << "Could not compute joint torques!" << RTT::endlog();
      this->error();
    }

    // Store the effort command
    joint_effort_ = torques_.data;

    // Send joint positions
    joint_effort_out_.write( joint_effort_ );

    // Debug visualization
    if(this->debug_throttle_.ready(0.05)) {
  
      if(!load_ok) {
        RTT::log(RTT::Warning) << "Attached inertia is overloading the controller." <<RTT::endlog();
      }

      wrench_msg_.header.frame_id = tip_link_;
      wrench_msg_.header.stamp = rtt_rosclock::host_now();
      wrench_msg_.wrench.force.x = ext_wrenches_.back().force.x();
      wrench_msg_.wrench.force.y = ext_wrenches_.back().force.y();
      wrench_msg_.wrench.force.z = ext_wrenches_.back().force.z();
      wrench_msg_.wrench.torque.x = ext_wrenches_.back().torque.x();
      wrench_msg_.wrench.torque.y = ext_wrenches_.back().torque.y();
      wrench_msg_.wrench.torque.z = ext_wrenches_.back().torque.z();
      ext_wrenches_debug_out_.write(wrench_msg_);

      for(std::vector<visualization_msgs::Marker>::iterator it = cogs_msgs_.begin();
          it != cogs_msgs_.end();
          ++it)
      {
        it->header.stamp = wrench_msg_.header.stamp;
        cogs_debug_out_.write(*it);
      }
    }
  }
}

void IDControllerKDL::stopHook()
{
}

void IDControllerKDL::cleanupHook()
{
}


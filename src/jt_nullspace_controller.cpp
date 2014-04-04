
// NOTE: These must be included before Eigen //
//#include <unsupported/Eigen/MatrixFunctions>
// ENDNOTE //

#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <rtt_rosparam/rosparam.h>

#include <rtt_tf/tf_interface.h>

#include <rtt_ros_tools/tools.h>
#include <rtt_roscomm/rtt_rostopic.h>
#include <rtt_rosclock/rtt_rosclock.h>
#include <kdl_urdf_tools/tools.h>

#include "jt_nullspace_controller.h"

using namespace lcsr_controllers;

JTNullspaceController::JTNullspaceController(std::string const& name) :
  RTT::TaskContext(name)
  // Properties
  ,robot_description_("")
  ,robot_description_param_("/robot_description")
  ,root_link_("")
  ,tip_link_("")
  ,target_frame_("")
  // Working variables
  ,n_dof_(0)
  // Params
  ,joint_center_gain_(0.0)
  ,singularity_avoidance_gain_(0.0)
  ,linear_p_gain_(0.0)
  ,linear_d_gain_(0.0)
  ,linear_effort_threshold_(0.0)
  ,linear_position_threshold_(0.0)
  ,linear_position_err_norm_(0.0)
  ,linear_effort_norm_(0.0)
  ,angular_p_gain_(0.0)
  ,angular_d_gain_(0.0)
  ,angular_effort_threshold_(0.0)
  ,angular_position_threshold_(0.0)
  ,angular_position_err_norm_(0.0)
  ,angular_effort_norm_(0.0)
  ,fk_solver_vel_(NULL)
  ,jac_solver_(NULL)
  ,chain_dynamics_(NULL)
  ,wrench_(6)
  // Throttles
  ,debug_throttle_(0.05)
  ,linear_position_within_tolerance_(false)
  ,linear_effort_within_tolerance_(false)
  ,angular_position_within_tolerance_(false)
  ,angular_effort_within_tolerance_(false)
  ,within_tolerance_(false)
  ,tf_(this)
{
  // Declare properties
  this->addProperty("robot_description",robot_description_)
    .doc("The WAM URDF xml string.");
  this->addProperty("robot_description_param",robot_description_param_)
    .doc("The ROS parameter name for the WAM URDF xml string.");
  this->addProperty("root_link",root_link_)
    .doc("The root link for the controller. Cartesian pose commands are given in this frame.");
  this->addProperty("tip_link",tip_link_)
    .doc("The tip link for the controller. Cartesian pose commands are applied to this frame.");
  this->addProperty("target_frame",target_frame_)
    .doc("The name of the target TF frame if tf is to be used. Leave blank to disable this.");

  this->addProperty("manipulability",manipulability_);
  this->addProperty("singularity_avoidance_gain",singularity_avoidance_gain_);
  this->addProperty("joint_center_gain",joint_center_gain_);
  this->addProperty("linear_p_gain",linear_p_gain_);
  this->addProperty("linear_d_gain",linear_d_gain_);
  this->addProperty("linear_position_threshold",linear_position_threshold_);
  this->addProperty("linear_effort_threshold",linear_effort_threshold_);
  this->addProperty("angular_p_gain",angular_p_gain_);
  this->addProperty("angular_d_gain",angular_d_gain_);
  this->addProperty("angular_position_threshold",angular_position_threshold_);
  this->addProperty("angular_effort_threshold",angular_effort_threshold_);

  this->addProperty("joint_d_gains",joint_d_gains_)
    .doc("Derivative gain used for joint-space control in the nullspace of the task-space command.");

  // Introspection
  this->addAttribute("linear_position_err_norm",linear_position_err_norm_);
  this->addAttribute("linear_effort_norm",linear_effort_norm_);
  this->addAttribute("angular_position_err_norm",angular_position_err_norm_);
  this->addAttribute("angular_effort_norm",angular_effort_norm_);
  this->addAttribute("wrench_raw",wrench_);
  this->addAttribute("joint_effort_raw",joint_effort_raw_);
  this->addAttribute("joint_effort_null_raw",joint_effort_null_);
  this->addAttribute("linear_position_within_tolerance",linear_position_within_tolerance_);
  this->addAttribute("linear_effort_within_tolerance",linear_effort_within_tolerance_);
  this->addAttribute("angular_position_within_tolerance",angular_position_within_tolerance_);
  this->addAttribute("angular_effort_within_tolerance",angular_effort_within_tolerance_);
  this->addAttribute("within_tolerance",within_tolerance_);

  // Configure data ports
  this->ports()->addPort("joint_position_in", joint_position_in_);
  this->ports()->addPort("joint_velocity_in", joint_velocity_in_);
  this->ports()->addPort("pose_twist_in", pose_twist_in_);
  this->ports()->addPort("joint_effort_out", joint_effort_out_);

  // Add the port and stream it to a ROS topic
  this->ports()->addPort("err_wrench_debug_out", err_wrench_debug_out_);
  err_wrench_debug_out_.createStream(rtt_roscomm::topic("~/"+this->getName()+"/err_wrench"));

  this->ports()->addPort("err_pose_debug_out", err_pose_debug_out_);
  err_pose_debug_out_.createStream(rtt_roscomm::topic("~/"+this->getName()+"/err_pose"));

  this->ports()->addPort("pose_desired_in", pose_desired_in_);
  pose_desired_in_.createStream(rtt_roscomm::topic("~/"+this->getName()+"/pose_desired"));
}

bool JTNullspaceController::configureHook()
{
  // ROS parameters
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = this->getProvider<rtt_rosparam::ROSParam>("rosparam");
  // Get private parameters
  rosparam->getComponentPrivate("root_link");
  rosparam->getComponentPrivate("tip_link");
  rosparam->getComponentPrivate("target_frame");
  rosparam->getComponentPrivate("singularity_avoidance_gain");
  rosparam->getComponentPrivate("joint_center_gain");
  rosparam->getComponentPrivate("linear_p_gain");
  rosparam->getComponentPrivate("linear_d_gain");
  rosparam->getComponentPrivate("linear_effort_threshold");
  rosparam->getComponentPrivate("linear_position_threshold");
  rosparam->getComponentPrivate("angular_p_gain");
  rosparam->getComponentPrivate("angular_d_gain");
  rosparam->getComponentPrivate("angular_effort_threshold");
  rosparam->getComponentPrivate("angular_position_threshold");

  rosparam->getComponentPrivate("joint_d_gains");

  rosparam->getComponentPrivate("robot_description_param");
  rosparam->getParam(robot_description_param_, "robot_description");
  if(robot_description_.length() == 0) {
    RTT::log(RTT::Error) << "No robot description! Reading from parameter \"" << robot_description_param_ << "\"" << RTT::endlog();
    return false;
  }

  if(!tf_.ready()) {
    RTT::log(RTT::Error) << this->getName() << " controller is not connected to tf!" << RTT::endlog(); 
    return false;
  }

  RTT::log(RTT::Debug) << "Initializing kinematic parameters from \"" << root_link_ << "\" to \"" << tip_link_ <<"\"" << RTT::endlog();

  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  urdf::Model urdf_model;
  if(!kdl_urdf_tools::initialize_kinematics_from_urdf(
        robot_description_, root_link_, tip_link_,
        n_dof_, kdl_chain_, kdl_tree_, urdf_model))
  {
    RTT::log(RTT::Error) << "Could not initialize robot kinematics!" << RTT::endlog();
    return false;
  }

  // Get joint limits
  {
    joint_limits_min_.resize(n_dof_);
    joint_limits_max_.resize(n_dof_);
    joint_limits_center_.resize(n_dof_);
    unsigned int i=0;
    for(std::vector<KDL::Segment>::const_iterator it=kdl_chain_.segments.begin();
        it != kdl_chain_.segments.end();
        it++)
    {
      joint_limits_min_(i) = urdf_model.joints_[it->getJoint().getName()]->limits->lower;
      joint_limits_max_(i) = urdf_model.joints_[it->getJoint().getName()]->limits->upper;
      joint_limits_center_(i) = (joint_limits_min_(i) + joint_limits_max_(i));
      i++;
    }
  }

  // Initialize IK solver
  fk_solver_vel_.reset(
      new KDL::ChainFkSolverVel_recursive(kdl_chain_));
  jac_solver_.reset(
      new KDL::ChainJntToJacSolver(kdl_chain_));
  chain_dynamics_.reset(
      new KDL::ChainDynParam(
          kdl_chain_, 
          KDL::Vector(0.0,0.0,0.0)));

  // Resize IO vectors
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_null_.resize(n_dof_);

  // Resize working vectors
  positions_.resize(n_dof_);
  velocities_.resize(n_dof_);
  posvel_.resize(n_dof_);
  jacobian_.resize(n_dof_);
  joint_inertia_.resize(n_dof_);
  joint_d_gains_.resize(n_dof_);

  // Prepare ports for realtime processing
  joint_effort_out_.setDataSample(joint_effort_);

  return true;
}

bool JTNullspaceController::startHook()
{
  // Reset tolerance flag (if only the world worked this way...)
  within_tolerance_ = true;

  return true;
}

void JTNullspaceController::updateHook()
{
  // Read in the current joint positions & velocities
  bool new_pos_data = joint_position_in_.readNewest( joint_position_ ) == RTT::NewData;
  bool new_vel_data = joint_velocity_in_.readNewest( joint_velocity_ ) == RTT::NewData;

  if(new_pos_data && new_vel_data) {
    // Get JntArray structures from pos/vel
    positions_.data = joint_position_;
    velocities_.data = joint_velocity_;
    
    posvel_.q = positions_;
    posvel_.qdot = velocities_;

    // Read the command
    RTT::FlowStatus ros_status = pose_desired_in_.readNewest(pose_msg_);
    RTT::FlowStatus rtt_status = pose_twist_in_.readNewest(framevel_desired_);

    if(ros_status == RTT::NewData) {
      KDL::Frame frame;
      tf::poseMsgToKDL(pose_msg_.pose, frame);
      framevel_desired_.M.R = frame.M;
      framevel_desired_.M.w = KDL::Vector::Zero();
      framevel_desired_.p.p = frame.p;
      framevel_desired_.p.v = KDL::Vector::Zero();
    } 
    else if(target_frame_.length() > 0) {
      if(tf_.canTransform(root_link_,target_frame_)) {
        geometry_msgs::TransformStamped tform_msg = tf_.lookupTransform(root_link_,target_frame_);
        KDL::Frame frame;
        tf::transformMsgToKDL(tform_msg.transform, frame);
        framevel_desired_.M.R = frame.M;
        framevel_desired_.M.w = KDL::Vector::Zero();
        framevel_desired_.p.p = frame.p;
        framevel_desired_.p.v = KDL::Vector::Zero();
      } else {
        RTT::log(RTT::Warning) << "Could not get the transform from "<<root_link_<<" to "<<target_frame_<<"."<<RTT::endlog();
        return;
      }
    }
    else if(rtt_status == RTT::NoData && ros_status == RTT::NoData) 
    {
      return;
    }

    // Compute forward kinematics of current pose
    // framevel_ is in the base_link coordinate frame
    fk_solver_vel_->JntToCart(posvel_, framevel_);

    // Compute the cartesian position and velocity error
    KDL::Frame frame = framevel_.GetFrame();
    KDL::Frame frame_des = framevel_desired_.GetFrame();
    KDL::Twist frame_err = KDL::diff(frame, frame_des);

    KDL::Twist twist = framevel_.GetTwist();
    KDL::Twist twist_des = framevel_desired_.GetTwist();
    KDL::Twist twist_err = KDL::diff(twist, twist_des);

    // Rotation error
    Eigen::Vector3d r_err = Eigen::Map<Eigen::Vector3d>(frame_err.rot.data);
    // Angular velocity error
    Eigen::Vector3d w_err = Eigen::Map<Eigen::Vector3d>(twist_err.rot.data);
    // Position error
    Eigen::Vector3d p_err = Eigen::Map<Eigen::Vector3d>(frame_err.vel.data);
    // Velocity error
    Eigen::Vector3d v_err = Eigen::Map<Eigen::Vector3d>(twist_err.vel.data);

    // Check pos/vel tolerances
    linear_position_err_norm_ = p_err.norm();
    angular_position_err_norm_ = r_err.norm();

    linear_position_within_tolerance_ = linear_position_err_norm_ < linear_position_threshold_;
    angular_position_within_tolerance_ = angular_position_err_norm_ < angular_position_threshold_;

    // Generalized force from PD control
    wrench_.segment<3>(0) = linear_p_gain_*p_err + linear_d_gain_*v_err;
    wrench_.segment<3>(3) = angular_p_gain_*r_err + angular_d_gain_*w_err;

    // Handy typedefs
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
    typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6Jd;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 6> MatrixJ6d;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixJJd;

    // Compute jacobian
    if(jac_solver_->JntToJac(positions_, jacobian_) != 0) {
      RTT::log(RTT::Error) << "Could not compute manipulator jacobian." << RTT::endlog();
      this->error();
      return;
    }
    Matrix6Jd J = jacobian_.data;
    MatrixJ6d J_t = J.transpose();

    // Compute the primary effort
    joint_effort_raw_ = J_t*wrench_;

    {
      // Compute nullspace effort (to be projected)
      joint_effort_null_.setZero();

      // Compute joint-space inertia matrix
      if(chain_dynamics_->JntToMass(positions_, joint_inertia_) != 0) {
        RTT::log(RTT::Error) << "Could not compute joint space inertia." << RTT::endlog();
        this->error();
        return;
      }
      Eigen::MatrixXd H = joint_inertia_.data;
      Eigen::MatrixXd H_inv = H.inverse();

      // Compute dynamically-consistent pseudoinverse
      MatrixJ6d J_pinv = H_inv * J_t * ( J * H_inv * J_t ).inverse();

      // Compute nullspace projector
      MatrixJJd eye(n_dof_,n_dof_);
      eye = Eigen::MatrixXd::Identity(n_dof_,n_dof_);
      MatrixJJd JpJ = (J_pinv * J);
      MatrixJJd N = eye - JpJ;
      MatrixJJd N_t = N.transpose();
      
      // Nullspace damping term //////////////////////////////////////////////////////////////////////
      {
        joint_effort_null_ += -1.0 * (joint_d_gains_.array() * joint_velocity_.array()).matrix();
      }

      // Singularity avoidance term //////////////////////////////////////////////////////////////////
      // Yoshikawa, 1984: "Analysis and Control of Robot Manipulators with Redundancy" ///////////////
      {
        // Compute manipulability
        Matrix6d G = J*J_t;
        Matrix6d G_inv = G.inverse();
        manipulability_ = sqrt(G.determinant());

        const double q_plus = 1E-4;
        KDL::Jacobian jac_plus(n_dof_);
        KDL::JntArray positions_plus(n_dof_);

        // Add the singularity avoidance from each joint component
        for(unsigned l=0; l < n_dof_; l++) {
          // Compute jacobian joint position derivative
          positions_plus.data = positions_.data;
          positions_plus.data(l) += q_plus;

          if(jac_solver_->JntToJac(positions_plus, jac_plus) != 0) {
            RTT::log(RTT::Error) << "Could not compute manipulator jacobian for manipulator jacobian derivative." << RTT::endlog();
            this->error();
            return;
          }
          Matrix6Jd dJdq = (jac_plus.data - J)/q_plus;

          for(unsigned i=0; i<6; i++) {
            for(unsigned j=0; j<6; j++) {
              joint_effort_null_(l) = 
                0.5*
                singularity_avoidance_gain_*
                manipulability_*
                G_inv(i,j)*
                (dJdq.row(i).dot(J.row(j)) + dJdq.row(j).dot(J.row(i)));
            }
          }
        }
      }

      // Joint limit avoidance term //////////////////////////////////////////////////////////////////
      {
        joint_effort_null_ += joint_center_gain_*(joint_limits_center_-joint_position_);
      }

      joint_effort_raw_ += N_t*joint_effort_null_;
    }

    // Project the hell out of it
    joint_effort_ = joint_effort_raw_;

    // Check effort tolerances
    linear_effort_norm_ = wrench_.block(0,0,3,1).norm();
    angular_effort_norm_ = wrench_.block(3,0,3,1).norm();
    linear_effort_within_tolerance_ = linear_effort_norm_ < linear_effort_threshold_;
    angular_effort_within_tolerance_ = angular_effort_norm_ < angular_effort_threshold_;

    // Safety: set output effor to zero if not within tolerances
    within_tolerance_ =
      within_tolerance_ &&
      linear_effort_within_tolerance_ &&
      linear_position_within_tolerance_ &&
      angular_effort_within_tolerance_ &&
      angular_position_within_tolerance_;

    if(!within_tolerance_) {
      joint_effort_.setZero();
    }

    // Send joint positions
    joint_effort_out_.write( joint_effort_ );

    // Debug visualization
    if(this->debug_throttle_.ready(0.05)) {
      wrench_msg_.header.frame_id = root_link_;
      wrench_msg_.header.stamp = rtt_rosclock::host_now();
      wrench_msg_.wrench.force.x = wrench_(0);
      wrench_msg_.wrench.force.y = wrench_(1);
      wrench_msg_.wrench.force.z = wrench_(2);
      wrench_msg_.wrench.torque.x = wrench_(3);
      wrench_msg_.wrench.torque.y = wrench_(4);
      wrench_msg_.wrench.torque.z = wrench_(5);
      err_wrench_debug_out_.write(wrench_msg_);

      KDL::Frame frame_err_(framevel_err_.M.R,framevel_err_.p.p);
      pose_err_msg_.header.frame_id = tip_link_;
      pose_err_msg_.header.stamp = rtt_rosclock::host_now();
      tf::poseKDLToMsg(frame_err_,pose_err_msg_.pose);
      err_pose_debug_out_.write(pose_err_msg_);
    }
  }
}

void JTNullspaceController::stopHook()
{
  joint_position_in_.clear();
  joint_velocity_in_.clear();
}

void JTNullspaceController::cleanupHook()
{
}


// NOTE: These must be included before Eigen //
//#include <unsupported/Eigen/MatrixFunctions>
// ENDNOTE //

#include <iostream>
#include <map>

#include <Eigen/Dense>
#include <Eigen/SVD> 

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <ocl/Component.hpp>

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
  ,singularity_avoidance_gain_(0.0)
  ,joint_center_gain_(0.0)
  ,nullspace_damping_(0.0)
  ,nullspace_min_singular_value_(0.00001)
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
  ,projector_type_(4)
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
  
  this->addOperation("printH",&JTNullspaceController::printH,this);

  this->addProperty("dur_get_data_",dur_get_data_);
  this->addProperty("dur_compute_wrench_",dur_compute_wrench_);
  this->addProperty("dur_compute_jac_",dur_compute_jac_);
  this->addProperty("dur_compute_eff_",dur_compute_eff_);
  this->addProperty("dur_compute_resize_nullspace_basis_",dur_compute_resize_nullspace_basis_);
  this->addProperty("dur_compute_nullspace_basis_",dur_compute_nullspace_basis_);
  this->addProperty("dur_compute_joint_inertia_",dur_compute_joint_inertia_);
  this->addProperty("dur_compute_nullspace_",dur_compute_nullspace_);
  this->addProperty("dur_compute_damping_",dur_compute_damping_);
  this->addProperty("dur_compute_singularity_avoidance_",dur_compute_singularity_avoidance_);

  this->addProperty("manipulability",manipulability_);
  this->addProperty("singularity_avoidance_gain",singularity_avoidance_gain_);
  this->addProperty("joint_center_gain",joint_center_gain_);
  this->addProperty("nullspace_damping",nullspace_damping_);
  this->addProperty("nullspace_min_singular_value",nullspace_min_singular_value_);
  this->addProperty("linear_p_gain",linear_p_gain_);
  this->addProperty("linear_d_gain",linear_d_gain_);
  this->addProperty("linear_position_threshold",linear_position_threshold_);
  this->addProperty("linear_effort_threshold",linear_effort_threshold_);
  this->addProperty("angular_p_gain",angular_p_gain_);
  this->addProperty("angular_d_gain",angular_d_gain_);
  this->addProperty("angular_position_threshold",angular_position_threshold_);
  this->addProperty("angular_effort_threshold",angular_effort_threshold_);
  this->addProperty("projector_type",projector_type_);

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
  this->ports()->addPort("joint_posture_in", joint_posture_in_);
  this->ports()->addPort("framevel_in", pose_twist_in_);
  this->ports()->addPort("joint_effort_out", joint_effort_out_);
  this->ports()->addPort("joint_velocity_des_out", joint_velocity_des_out_);

  // Add the port and stream it to a ROS topic
  this->ports()->addPort("err_wrench_debug_out", err_wrench_debug_out_);
  err_wrench_debug_out_.createStream(rtt_roscomm::topic("~/"+this->getName()+"/err_wrench"));

  this->ports()->addPort("err_pose_debug_out", err_pose_debug_out_);
  err_pose_debug_out_.createStream(rtt_roscomm::topic("~/"+this->getName()+"/err_pose"));

  this->ports()->addPort("pose_desired_in", pose_desired_in_);
  pose_desired_in_.createStream(rtt_roscomm::topic("~/"+this->getName()+"/pose_desired"));

  this->ports()->addPort("joint_state_des_debug_out", effort_debug_out_);
  effort_debug_out_.createStream(rtt_roscomm::topic("~/"+this->getName()+"/joint_state"));
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
  rosparam->getComponentPrivate("nullspace_damping");
  rosparam->getComponentPrivate("linear_p_gain");
  rosparam->getComponentPrivate("linear_d_gain");
  rosparam->getComponentPrivate("linear_effort_threshold");
  rosparam->getComponentPrivate("linear_position_threshold");
  rosparam->getComponentPrivate("angular_p_gain");
  rosparam->getComponentPrivate("angular_d_gain");
  rosparam->getComponentPrivate("angular_effort_threshold");
  rosparam->getComponentPrivate("angular_position_threshold");

  rosparam->getComponentPrivate("joint_d_gains");
  rosparam->getComponentPrivate("projector_type");

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
    joint_limits_range_.resize(n_dof_);
    unsigned int i=0;
    for(std::vector<KDL::Segment>::const_iterator it=kdl_chain_.segments.begin();
        it != kdl_chain_.segments.end();
        it++)
    {
      if(it->getJoint().getType() != KDL::Joint::None) {
        joint_limits_min_(i) = urdf_model.joints_[it->getJoint().getName()]->limits->lower;
        joint_limits_max_(i) = urdf_model.joints_[it->getJoint().getName()]->limits->upper;
        joint_limits_center_(i) = (joint_limits_min_(i) + joint_limits_max_(i))/2.0;
        joint_limits_range_(i) = (joint_limits_max_(i) - joint_limits_min_(i));
        i++;
      }
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
  wrench_.resize(6);
  twist_.resize(6);
  velocities_.resize(n_dof_);
  posvel_.resize(n_dof_);
  jacobian_.resize(n_dof_);
  joint_inertia_.resize(n_dof_);
  joint_d_gains_.resize(n_dof_);
  joint_velocity_des_.resize(n_dof_);

  joint_state_msg_.position.resize(n_dof_);
  joint_state_msg_.velocity.resize(n_dof_);
  joint_state_msg_.effort.resize(n_dof_);

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
  //bool new_posture_data = joint_posture_in_.readNewest( joint_posture_ ) == RTT::NewData;

  RTT::os::TimeService *ts =  RTT::os::TimeService::Instance();

  RTT::os::TimeService::ticks zero = 0;
  RTT::os::TimeService::ticks tic;

  if(new_pos_data && new_vel_data) {

    tic = ts->getTicks();

    // Get JntArray structures from pos/vel
    posvel_.q.data = joint_position_;
    posvel_.qdot.data = joint_velocity_;

    // Read the command
    RTT::FlowStatus ros_status = pose_desired_in_.readNewest(pose_msg_);
    RTT::FlowStatus rtt_status = pose_twist_in_.readNewest(framevel_desired_);

    if(ros_status == RTT::NewData) 
    {
      KDL::Frame frame;
      tf::poseMsgToKDL(pose_msg_.pose, frame);
      framevel_desired_.M.R = frame.M;
      framevel_desired_.M.w = KDL::Vector::Zero();
      framevel_desired_.p.p = frame.p;
      framevel_desired_.p.v = KDL::Vector::Zero();
    } 
    else if(target_frame_.length() > 0) 
    {
      if(tf_.canTransform(root_link_,target_frame_)) {
        geometry_msgs::TransformStamped tform_msg = tf_.lookupTransform(root_link_, target_frame_);
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

    dur_get_data_ = ts->secondsSince(tic);
    tic = ts->getTicks();

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

    // Store eigen twist
    for(unsigned i=0;i<3;i++) {
      twist_(i) = twist.vel.data[i];
      twist_(i+3) = twist.rot.data[i];
    }

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

    dur_compute_wrench_ = ts->secondsSince(tic);
    tic = ts->getTicks();


    // Compute jacobian
    if(jac_solver_->JntToJac(posvel_.q, jacobian_) != 0) {
      RTT::log(RTT::Error) << "Could not compute manipulator jacobian." << RTT::endlog();
      this->error();
      return;
    }
    
    Matrix6Jd J = jacobian_.data;
    MatrixJ6d J_t = J.transpose();

    dur_compute_jac_ = ts->secondsSince(tic);
    tic = ts->getTicks();

    // Compute the primary effort
    joint_effort_raw_ = J_t*wrench_;

    dur_compute_eff_ = ts->secondsSince(tic);
      tic = ts->getTicks();

    Eigen::HouseholderQR<MatrixJ6d> J_t_qr;
    J_t_qr.compute(J_t);

    // Compute nullspace effort (to be projected)
    // This is based on:
    // Springer Tracts in Advanced Robotics: Volume 49 
    // Cartesian Impedance Control of Redundant and Flexible-Joint Robots
    // by Christian Ott (ISBN 978-3-540-69253-9)
    {
      joint_effort_null_.setZero();

      // Compute nullspace basis
      // J_t = QR --> gives nullspace of J
      Z = Eigen::MatrixXd(J_t_qr.householderQ()).rightCols(n_dof_-6).transpose();

      dur_compute_nullspace_basis_ = ts->secondsSince(tic);
      tic = ts->getTicks();

      // Compute joint-space inertia matrix
      if(projector_type_ > 1) {
        if(chain_dynamics_->JntToMass(posvel_.q, joint_inertia_) != 0) {
          RTT::log(RTT::Error) << "Could not compute joint space inertia." << RTT::endlog();
          this->error();
          return;
        }
      }

      dur_compute_joint_inertia_ = ts->secondsSince(tic);
      tic = ts->getTicks();

      // Compute projector
      // Smallest singular value
      double s_min;
      switch(projector_type_) {
        case 1: { // Unweighted projector
                  P1 = Z.transpose()*(Z*Z.transpose()).inverse()*Z;
                  N = P1;
                  break; }
        case 2: { // Mass-Weighted projector (to scale)
                  P2 = (joint_inertia_.data)*Z.transpose()*(Z*Z.transpose()).inverse()*Z;
                  N = P2;
                  break; }
        case 3: { // Dynamically consistent projector
                  Eigen::MatrixXd &M = (joint_inertia_.data); // + (eye + d_gain).inverse() * motor_inertia 
                  P3 = M*Z.transpose()*(Z*M*Z.transpose()).inverse()*Z;
                  N = P3;
                  break; }
        case 4: { // Operational space dynamically consistent projector
                  Eigen::MatrixXd &M = (joint_inertia_.data); // + (eye + d_gain).inverse() * motor_inertia 
                  Eigen::MatrixXd M_inv = M.inverse();
                  MatrixJJd eye = MatrixJJd::Identity(n_dof_,n_dof_);
                  Matrix6d JMinvJt = J*M_inv*J_t;
                  Eigen::JacobiSVD<Matrix6d> svd(JMinvJt, Eigen::ComputeFullU | Eigen::ComputeFullV);
                  Eigen::VectorXd s = svd.singularValues();
                  Matrix6d S_inv(Matrix6d::Zero());
                  for(int i=0; i<s.size(); i++) {
                    if(s(i) > 0.01) {
                      S_inv(i,i) = 1.0/s(i);
                    } else {
                      S_inv(i,i) = 0.0;
                    }
                  }
                  Matrix6d JMinvJt_inv = svd.matrixV() * S_inv * svd.matrixU().transpose();
                  P4 = eye - J_t*(JMinvJt_inv)*J*M_inv;
                  N = P4;
                  break; }
      };
      
      if(s_min < 0.001) {
        //RTT::log(RTT::Warning) << "Minimum singular value in projector is: "<<s_min<<RTT::endlog();
      }

      dur_compute_nullspace_ = ts->secondsSince(tic);
      tic = ts->getTicks();

      // Nullspace damping term 
      {
        joint_effort_null_ -= nullspace_damping_ * (joint_d_gains_.array() * joint_velocity_.array()).matrix();
      }

      // Nullspace limit avoidance term
      // Local gradient optimization based on cost fucntion given in "Inverse
      // Kinematics and Control of a 7-DOF Redundant Manipulator Based on the
      // Closed-Loop Algorithm"
      {
        Eigen::VectorXd center_err_by_range = ((joint_position_ - joint_limits_center_).array() / joint_limits_range_.array()).matrix();
        Eigen::VectorXd center_direction = (center_err_by_range.array() * center_err_by_range.array().abs().pow(4)
          / center_err_by_range.lpNorm<6>()).matrix();

        joint_effort_null_ -= joint_center_gain_ * center_direction;
      }

      dur_compute_damping_ = ts->secondsSince(tic);
      tic = ts->getTicks();

      // Singularity avoidance term //////////////////////////////////////////////////////////////////
      // Yoshikawa, 1984: "Analysis and Control of Robot Manipulators with Redundancy" ///////////////
      // TODO: Optimize this computation for realtime
      if(singularity_avoidance_gain_ > 0.001)
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
          positions_plus.data = posvel_.q.data;
          positions_plus.data(l) += q_plus;

          if(jac_solver_->JntToJac(positions_plus, jac_plus) != 0) {
            RTT::log(RTT::Error) << "Could not compute manipulator jacobian for manipulator jacobian derivative." << RTT::endlog();
            this->error();
            return;
          }
          Matrix6Jd dJdq = (jac_plus.data - J)/q_plus;

          for(unsigned i=0; i<6; i++) {
            for(unsigned j=0; j<6; j++) {
              joint_effort_null_(l) += 
                0.5*
                singularity_avoidance_gain_*
                manipulability_*
                G_inv(i,j)*
                (dJdq.row(i).dot(J.row(j)) + dJdq.row(j).dot(J.row(i)));
            }
          }
        }
      }

      dur_compute_singularity_avoidance_ = ts->secondsSince(tic);
      tic = ts->getTicks();

      joint_effort_raw_ += N*joint_effort_null_;
    }

    // Project the hell out of it
    joint_effort_ = joint_effort_raw_;

    // Check effort tolerances
    linear_effort_norm_ = wrench_.block(0,0,3,1).norm();
    angular_effort_norm_ = wrench_.block(3,0,3,1).norm();
    linear_effort_within_tolerance_ = linear_effort_norm_ < linear_effort_threshold_;
    angular_effort_within_tolerance_ = angular_effort_norm_ < angular_effort_threshold_;

    if(within_tolerance_ && !linear_effort_within_tolerance_) {
      RTT::log(RTT::Warning) << "JTNS: Linear effort exceeded tolerance: " << linear_effort_norm_ <<RTT::endlog();
    }
    if(within_tolerance_ && !angular_effort_within_tolerance_) {
      RTT::log(RTT::Warning) << "JTNS: Angular effort exceeded tolerance: " << angular_effort_norm_ <<RTT::endlog();
    }

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

    // Send joint efforts
    joint_effort_out_.write( joint_effort_ );

    // Compute desired velocity
    Eigen::MatrixXd Ja(n_dof_, n_dof_), Ja_inv(n_dof_, n_dof_);
    Eigen::VectorXd twist_a_(n_dof_);

#if 0
    Ja.topRows(6) = J;
    Ja.bottomRows(n_dof_-6) = Z;
    twist_a_.head(6) = twist_;
    twist_a_.tail(n_dof_-6).setZero();

    joint_velocity_des_ = Ja.householderQr().solve(twist_a_);
#endif

    joint_velocity_des_out_.write( joint_velocity_des_ );

    // Debug visualization
    if(this->debug_throttle_.ready(0.05)) {
      wrench_msg_.header.frame_id = tip_link_;
      wrench_msg_.header.stamp = rtt_rosclock::host_now();
      KDL::Wrench tip_wrench = frame.Inverse()*KDL::Wrench(KDL::Vector(wrench_(0), wrench_(1), wrench_(2)), KDL::Vector(wrench_(3), wrench_(4), wrench_(5)));
      wrench_msg_.wrench.force.x = tip_wrench(0);
      wrench_msg_.wrench.force.y = tip_wrench(1);
      wrench_msg_.wrench.force.z = tip_wrench(2);
      wrench_msg_.wrench.torque.x = tip_wrench(3);
      wrench_msg_.wrench.torque.y = tip_wrench(4);
      wrench_msg_.wrench.torque.z = tip_wrench(5);
      err_wrench_debug_out_.write(wrench_msg_);

      KDL::Frame frame_err_(framevel_err_.M.R,framevel_err_.p.p);

      pose_err_msg_.header.frame_id = tip_link_;
      pose_err_msg_.header.stamp = rtt_rosclock::host_now();
      tf::poseKDLToMsg(frame_err_,pose_err_msg_.pose);
      err_pose_debug_out_.write(pose_err_msg_);

      joint_state_msg_.header.frame_id = root_link_;
      joint_state_msg_.header.stamp = rtt_rosclock::host_now();
      for(unsigned i=0; i < n_dof_; i++) {
        joint_state_msg_.velocity[i] = joint_velocity_des_(i);
        joint_state_msg_.effort[i] = joint_effort_(i);
      }
      effort_debug_out_.write(joint_state_msg_);
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

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(lcsr_controllers::JTNullspaceController)

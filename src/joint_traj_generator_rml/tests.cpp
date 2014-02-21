
#include <string>
#include <vector>
#include <iterator>

#include <rtt/os/startstop.h>

#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>
#include <rtt/deployment/ComponentLoader.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>

#include <trajectory_msgs/JointTrajectory.h>

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <gtest/gtest.h>
#include <gmock/gmock.h>
using ::testing::ElementsAre;

#include "joint_traj_generator_rml.h"
using namespace lcsr_controllers;

#include <ros/ros.h>

#include <rtt_ros/ros.h>

class StaticTest : public ::testing::Test {
public:
  size_t n_dof;
  double t_step;
  size_t n_base_traj_points;
  trajectory_msgs::JointTrajectory traj_msg;
  ros::Time now;

  StaticTest() :
    ::testing::Test(),
    t_step(1.0),
    n_base_traj_points(10),
    n_dof(7),
    traj_msg(),
    now(1000,1000)
  { 
    // create a sinusoidal trajectory
    for(int i=0; i <n_base_traj_points; i++) {
      trajectory_msgs::JointTrajectoryPoint point;

      double t = 1.0 + i*t_step;

      point.time_from_start = ros::Duration(t);
      point.positions.resize(n_dof);
      point.velocities.resize(n_dof);

      for(int n=0; n<n_dof; n++) {
        point.positions[n] = sin(n*2.0*M_PI*t);
        point.velocities[n] = cos(n*2.0*M_PI*t);
      }

      traj_msg.points.push_back(point);
    }
  }
};

TEST_F(StaticTest, NowMsgConversion) 
{
  // convert the trajectory message into a list of trajectory segments
  JointTrajGeneratorRML::TrajSegments segments;
  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      traj_msg,
      n_dof,
      now,
      segments);

  // Verify that the right number of points were copied
  EXPECT_EQ(segments.size(),n_base_traj_points);
  ASSERT_EQ(segments.size(),traj_msg.points.size());

  // Verify that everything got copied properly
  JointTrajGeneratorRML::TrajSegments::const_iterator segment_it = segments.begin(); 
  for(int i=0; i<segments.size(); i++) {
    ASSERT_NE(segment_it, segments.end());

    EXPECT_EQ(segment_it->goal_time, now + traj_msg.points[i].time_from_start);

    ASSERT_EQ(traj_msg.points[i].positions.size(), segment_it->goal_positions.size());
    ASSERT_EQ(traj_msg.points[i].velocities.size(), segment_it->goal_velocities.size());

    for(int n=0; n<n_dof; n++) {
      EXPECT_EQ(traj_msg.points[i].positions[n],segment_it->goal_positions[n]);
      EXPECT_EQ(traj_msg.points[i].velocities[n],segment_it->goal_velocities[n]);
    }
    segment_it++;
  }
}

TEST_F(StaticTest, SpliceLaterTrajectory) 
{
  JointTrajGeneratorRML::TrajSegments segments_current, segments_new;
  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      traj_msg,
      n_dof,
      now,
      segments_current);
  EXPECT_EQ(segments_current.front().start_time, now);

  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      traj_msg,
      n_dof,
      now + ros::Duration(10.0),
      segments_new);
  EXPECT_EQ(segments_new.front().start_time, now + ros::Duration(10.0));

  JointTrajGeneratorRML::UpdateTrajectory(segments_current, segments_new);

  EXPECT_EQ(segments_current.size(),2*n_base_traj_points); 
}

TEST_F(StaticTest, SpliceEarlierTrajectory) 
{
  JointTrajGeneratorRML::TrajSegments segments_current, segments_new;
  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      traj_msg,
      n_dof,
      now,
      segments_current);
  EXPECT_EQ(segments_current.front().start_time, now);

  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      traj_msg,
      n_dof,
      now - ros::Duration(5.0),
      segments_new);
  EXPECT_EQ(segments_new.front().start_time, now - ros::Duration(5.0));

  JointTrajGeneratorRML::UpdateTrajectory(segments_current, segments_new);

  EXPECT_EQ(segments_current.size(),n_base_traj_points); 
}

TEST_F(StaticTest, SpliceInterruptingTrajectory) 
{
  JointTrajGeneratorRML::TrajSegments segments_current, segments_new;
  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      traj_msg,
      n_dof,
      now,
      segments_current);

  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      traj_msg,
      n_dof,
      now + ros::Duration(5.0),
      segments_new);

  JointTrajGeneratorRML::UpdateTrajectory(segments_current, segments_new);

  EXPECT_EQ(segments_current.size(),1.5*n_base_traj_points); 
}

class InstanceTest : public StaticTest 
{
public:

  boost::shared_ptr<JointTrajGeneratorRML> task;

  double sampling_resolution;
  Eigen::VectorXd 
    position_tolerance,
    max_velocities,
    max_accelerations,
    max_jerks;

  boost::shared_ptr<ReflexxesAPI> rml;
  boost::shared_ptr<RMLPositionInputParameters> rml_in;
  boost::shared_ptr<RMLPositionOutputParameters> rml_out;
  RMLPositionFlags rml_flags;

  InstanceTest() :
    StaticTest(),
    task(new JointTrajGeneratorRML("test_traj_rml")),
    sampling_resolution(0.001),
    max_velocities(Eigen::VectorXd::Constant(n_dof,2.5)),
    max_accelerations(Eigen::VectorXd::Constant(n_dof,5.0)),
    max_jerks(Eigen::VectorXd::Constant(n_dof,1.0))
  {
    task->n_dof_ = n_dof;
    task->sampling_resolution_ = sampling_resolution;
    task->position_tolerance_ = position_tolerance;
    task->max_velocities_ = max_velocities;
    task->max_accelerations_ = max_accelerations;
    task->max_jerks_ = max_jerks;
  }
};

TEST_F(InstanceTest, Configure)
{
  ASSERT_TRUE(task->configure());
  ASSERT_TRUE(task->configureRML(rml, rml_in, rml_out, rml_flags));
}

TEST_F(InstanceTest, EmptyTraj)
{
  RecordProperty("description", 
                 "This tests the trajectory generator with an empty trajectory. "
                 "It should do nothing.");

  ASSERT_TRUE(task->configure());
  ASSERT_TRUE(task->configureRML(rml, rml_in, rml_out, rml_flags));

  JointTrajGeneratorRML::TrajSegments segments;

  Eigen::VectorXd
    joint_position(n_dof),
    joint_velocity(n_dof),
    joint_position_sample(n_dof),
    joint_velocity_sample(n_dof),
    joint_position_sample_expected(n_dof),
    joint_velocity_sample_expected(n_dof);

  joint_position << 0,0,0,0,0,0,0;
  joint_velocity << 0,0,0,0,0,0,0;
  joint_position_sample << 1,2,3,4,5,6,7;
  joint_velocity_sample << 1,2,3,4,5,6,7;

  joint_position_sample_expected = joint_position_sample;
  joint_velocity_sample_expected = joint_velocity_sample;

  // This should return false and shouldn't modify the samples
  EXPECT_FALSE(
      task->sampleTrajectory(
          now,
          joint_position, joint_velocity,
          rml, rml_in, rml_out, rml_flags,
          segments,
          joint_position_sample, joint_velocity_sample));

  EXPECT_EQ(segments.size(),0);
  EXPECT_TRUE(joint_position_sample == joint_position_sample_expected);
  EXPECT_TRUE(joint_velocity_sample == joint_velocity_sample_expected);
}

TEST_F(InstanceTest, OldTraj)
{
  RecordProperty("description", 
                 "This tests the trajectory generator with a trajectory which "
                 "should have already been completed. It should clear the "
                 "trajectory and generate no new samples.");

  JointTrajGeneratorRML::TrajSegments segments;
  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      traj_msg,
      n_dof,
      now - ros::Duration(20.0),
      segments);

  ASSERT_TRUE(task->configure());
  ASSERT_TRUE(task->configureRML(rml, rml_in, rml_out, rml_flags));

  Eigen::VectorXd
    joint_position(n_dof),
    joint_velocity(n_dof),
    joint_position_sample(n_dof),
    joint_velocity_sample(n_dof),
    joint_position_sample_expected(n_dof),
    joint_velocity_sample_expected(n_dof);

  joint_position << 0,0,0,0,0,0,0;
  joint_velocity << 0,0,0,0,0,0,0;
  joint_position_sample << 1,2,3,4,5,6,7;
  joint_velocity_sample << 1,2,3,4,5,6,7;

  joint_position_sample_expected = joint_position_sample;
  joint_velocity_sample_expected = joint_velocity_sample;

  // This should return false and shouldn't modify the samples
  EXPECT_FALSE(
      task->sampleTrajectory(
          now,
          joint_position, joint_velocity,
          rml, rml_in, rml_out, rml_flags,
          segments,
          joint_position_sample, joint_velocity_sample));

  EXPECT_EQ(segments.size(),0);
  EXPECT_TRUE(joint_position_sample == joint_position_sample_expected);
  EXPECT_TRUE(joint_velocity_sample == joint_velocity_sample_expected);
}

TEST_F(InstanceTest, LateTraj)
{
  RecordProperty("description", 
                 "This tests the trajectory generator with a trajectory which "
                 "started before now, but still has points that were not yet "
                 "meant to be started. It should generate a sample and it "
                 "should indicate the head trajectory segment is active." );
  JointTrajGeneratorRML::TrajSegments segments;
  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      traj_msg,
      n_dof,
      now - ros::Duration(5.0),
      segments);

  ASSERT_TRUE(task->configure());
  ASSERT_TRUE(task->configureRML(rml, rml_in, rml_out, rml_flags));

  Eigen::VectorXd
    joint_position(n_dof),
    joint_velocity(n_dof),
    joint_position_sample(n_dof),
    joint_velocity_sample(n_dof),
    joint_position_sample_expected(n_dof),
    joint_velocity_sample_expected(n_dof);

  joint_position << 0,0,0,0,0,0,0;
  joint_velocity << 0,0,0,0,0,0,0;
  joint_position_sample << 1,2,3,4,5,6,7;
  joint_velocity_sample << 1,2,3,4,5,6,7;

  joint_position_sample_expected = joint_position_sample;
  joint_velocity_sample_expected = joint_velocity_sample;

  // This should return false and shouldn't modify the samples
  EXPECT_TRUE(
      task->sampleTrajectory(
          now,
          joint_position, joint_velocity,
          rml, rml_in, rml_out, rml_flags,
          segments,
          joint_position_sample, joint_velocity_sample));

  ASSERT_EQ(segments.size(),5);
  EXPECT_TRUE(segments.front().active);
  EXPECT_TRUE(joint_position_sample != joint_position_sample_expected);
  EXPECT_TRUE(joint_velocity_sample != joint_velocity_sample_expected);
}

TEST_F(InstanceTest, NowTraj)
{
  RecordProperty("description", 
                 "This tests the trajectory generator with a trajectory which "
                 "starts immediately. It should generate a sample and it should "
                 "indicate the head trajectory segment is active." );

  JointTrajGeneratorRML::TrajSegments segments;
  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      traj_msg,
      n_dof,
      now,
      segments);

  ASSERT_TRUE(task->configure());
  ASSERT_TRUE(task->configureRML(rml, rml_in, rml_out, rml_flags));

  Eigen::VectorXd
    joint_position(n_dof),
    joint_velocity(n_dof),
    joint_position_sample(n_dof),
    joint_velocity_sample(n_dof),
    joint_position_sample_original(n_dof),
    joint_velocity_sample_original(n_dof);

  joint_position << 0,0,0,0,0,0,0;
  joint_velocity << 0,0,0,0,0,0,0;
  joint_position_sample << 1,2,3,4,5,6,7;
  joint_velocity_sample << 1,2,3,4,5,6,7;

  joint_position_sample_original = joint_position_sample;
  joint_velocity_sample_original = joint_velocity_sample;

  // This should return false and shouldn't modify the samples
  EXPECT_TRUE(
      task->sampleTrajectory(
          now,
          joint_position, joint_velocity,
          rml, rml_in, rml_out, rml_flags,
          segments,
          joint_position_sample, joint_velocity_sample));

  ASSERT_EQ(segments.size(),n_base_traj_points);
  EXPECT_TRUE(segments.front().active);
  EXPECT_TRUE(joint_position_sample != joint_position_sample_original);
  EXPECT_TRUE(joint_velocity_sample != joint_velocity_sample_original);
}

TEST_F(InstanceTest, SoonTraj)
{
  RecordProperty("description", 
                 "This tests the trajectory generator with a trajectory which "
                 "starts in the future. It should not generate any samples, and "
                 "it shouldn't modify the trajectory at all." );

  JointTrajGeneratorRML::TrajSegments segments;
  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      traj_msg,
      n_dof,
      now + ros::Duration(10.0),
      segments);

  ASSERT_TRUE(task->configure());
  ASSERT_TRUE(task->configureRML(rml, rml_in, rml_out, rml_flags));

  Eigen::VectorXd
    joint_position(n_dof),
    joint_velocity(n_dof),
    joint_position_sample(n_dof),
    joint_velocity_sample(n_dof),
    joint_position_sample_original(n_dof),
    joint_velocity_sample_original(n_dof);

  joint_position << 0,0,0,0,0,0,0;
  joint_velocity << 0,0,0,0,0,0,0;
  joint_position_sample << 1,2,3,4,5,6,7;
  joint_velocity_sample << 1,2,3,4,5,6,7;

  joint_position_sample_original = joint_position_sample;
  joint_velocity_sample_original = joint_velocity_sample;

  // This should return false and shouldn't modify the samples
  EXPECT_FALSE(
      task->sampleTrajectory(
          now,
          joint_position, joint_velocity,
          rml, rml_in, rml_out, rml_flags,
          segments,
          joint_position_sample, joint_velocity_sample));

  EXPECT_EQ(segments.size(),n_base_traj_points);
  EXPECT_FALSE(segments.front().active);
  EXPECT_TRUE(joint_position_sample == joint_position_sample_original);
  EXPECT_TRUE(joint_velocity_sample == joint_velocity_sample_original);
}

TEST_F(InstanceTest, UnaryTraj)
{
  RecordProperty("description", 
                 "This tests the trajectory generator with a trajectory which "
                 "starts in the future. It should not generate any samples, and "
                 "it shouldn't modify the trajectory at all." );

  
  // Create a unary trajectory
  trajectory_msgs::JointTrajectory unary_joint_traj;
  trajectory_msgs::JointTrajectoryPoint unary_joint_traj_point;
  unary_joint_traj_point.positions.assign(n_dof,1.0);
  unary_joint_traj.points.push_back(unary_joint_traj_point);

  JointTrajGeneratorRML::TrajSegments segments;
  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      unary_joint_traj,
      n_dof,
      now,
      segments);

  ASSERT_TRUE(task->configure());
  ASSERT_TRUE(task->configureRML(rml, rml_in, rml_out, rml_flags));

  Eigen::VectorXd
    joint_position(n_dof),
    joint_velocity(n_dof),
    joint_position_sample(n_dof),
    joint_velocity_sample(n_dof),
    joint_position_sample_original(n_dof),
    joint_velocity_sample_original(n_dof);

  joint_position << 0,0,0,0,0,0,0;
  joint_velocity << 0,0,0,0,0,0,0;
  joint_position_sample << 1,2,3,4,5,6,7;
  joint_velocity_sample << 1,2,3,4,5,6,7;

  joint_position_sample_original = joint_position_sample;
  joint_velocity_sample_original = joint_velocity_sample;

  // This should return false and shouldn't modify the samples
  ASSERT_EQ(segments.size(),1);
  EXPECT_TRUE(
      task->sampleTrajectory(
          now,
          joint_position, joint_velocity,
          rml, rml_in, rml_out, rml_flags,
          segments,
          joint_position_sample, joint_velocity_sample));

  ASSERT_EQ(segments.size(),1);
  EXPECT_TRUE(segments.front().active);
  EXPECT_TRUE(joint_position_sample != joint_position_sample_original);
  EXPECT_TRUE(joint_velocity_sample != joint_velocity_sample_original);
}


TEST_F(InstanceTest, FullTraj)
{
  RecordProperty("description", 
                 "This tests the trajectory generator with a trajectory which "
                 "starts in the future. It should not generate any samples, and "
                 "it shouldn't modify the trajectory at all." );

  JointTrajGeneratorRML::TrajSegments segments;
  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      traj_msg,
      n_dof,
      now,
      segments);

  ASSERT_TRUE(task->configure());
  ASSERT_TRUE(task->configureRML(rml, rml_in, rml_out, rml_flags));

  Eigen::VectorXd
    joint_position(n_dof),
    joint_velocity(n_dof),
    joint_position_sample(n_dof),
    joint_velocity_sample(n_dof),
    joint_position_sample_original(n_dof),
    joint_velocity_sample_original(n_dof);

  joint_position << 0,0,0,0,0,0,0;
  joint_velocity << 0,0,0,0,0,0,0;
  joint_position_sample << 1,2,3,4,5,6,7;
  joint_velocity_sample << 1,2,3,4,5,6,7;

  joint_position_sample_original = joint_position_sample;
  joint_velocity_sample_original = joint_velocity_sample;

  bool sampled_traj = true;
  ros::Time looptime = now;
  while(sampled_traj) {
    sampled_traj = task->sampleTrajectory(
        looptime,
        joint_position, joint_velocity,
        rml, rml_in, rml_out, rml_flags,
        segments,
        joint_position_sample, joint_velocity_sample);
    looptime = looptime + ros::Duration(0.002);
    joint_position = joint_position_sample;
    joint_velocity = joint_velocity_sample;
  }

  EXPECT_EQ(segments.size(),0);
}

TEST_F(InstanceTest, FlexibleTraj)
{
  RecordProperty("description", 
                 "This tests the trajectory generator with a trajectory which "
                 "starts in the future. It should not generate any samples, and "
                 "it shouldn't modify the trajectory at all." );

  JointTrajGeneratorRML::TrajSegments segments;
  trajectory_msgs::JointTrajectory flexible_traj_msg = traj_msg;
  for(std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator it = flexible_traj_msg.points.begin();
      it != flexible_traj_msg.points.end();
      ++it)
  {
    it->time_from_start = ros::Duration(0,0);
  }

  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      flexible_traj_msg,
      n_dof,
      now,
      segments);

  ASSERT_TRUE(task->configure());
  ASSERT_TRUE(task->configureRML(rml, rml_in, rml_out, rml_flags));

  Eigen::VectorXd
    joint_position(n_dof),
    joint_velocity(n_dof),
    joint_position_sample(n_dof),
    joint_velocity_sample(n_dof),
    joint_position_sample_original(n_dof),
    joint_velocity_sample_original(n_dof);

  joint_position << 0,0,0,0,0,0,0;
  joint_velocity << 0,0,0,0,0,0,0;
  joint_position_sample << 1,2,3,4,5,6,7;
  joint_velocity_sample << 1,2,3,4,5,6,7;

  joint_position_sample_original = joint_position_sample;
  joint_velocity_sample_original = joint_velocity_sample;

  bool sampled_traj = true;
  ros::Time looptime = now;
  while(sampled_traj) {
    sampled_traj = task->sampleTrajectory(
        looptime,
        joint_position, joint_velocity,
        rml, rml_in, rml_out, rml_flags,
        segments,
        joint_position_sample, joint_velocity_sample);
    looptime = looptime + ros::Duration(0.002);
    joint_position = joint_position_sample;
    joint_velocity = joint_velocity_sample;
  }

  EXPECT_EQ(segments.size(),0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  // Initialize Orocos
  __os_init(argc, argv);

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  //RTT::Logger::log().setLogLevel(RTT::Logger::Debug);

  // Import conman plugin
  if(!RTT::ComponentLoader::Instance()->import("conman", "" )) {
    std::cerr<<"Could not import conman package."<<std::endl;
    return -1;
  }
  if(!RTT::ComponentLoader::Instance()->import("rtt_ros", "" )) {
    std::cerr<<"Could not import rtt_ros package."<<std::endl;
    return -1;
  }
  rtt_ros::ROS ros_requester;
  ros_requester.import("rtt_rosparam");
  ros_requester.import("lcsr_controllers");
  
  return RUN_ALL_TESTS();
}

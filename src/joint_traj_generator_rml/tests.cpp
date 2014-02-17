
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

class StaticTest : public ::testing::Test {
public:
  size_t n_dof;
  trajectory_msgs::JointTrajectory traj_msg;
  ros::Time now;

  StaticTest() :
    ::testing::Test(),
    n_dof(7),
    traj_msg(),
    now(1000,1000)
  { 
    // create a sinusoidal trajectory
    for(double t=0; t < 1.0; t+=0.2) {
      trajectory_msgs::JointTrajectoryPoint point;
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
  EXPECT_EQ(segments.size(),5);
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

#if 0
TEST_F(StaticTest, TrajectorySplicing) 
{
  // convert the trajectory message into a list of trajectory segments
  JointTrajGeneratorRML::TrajSegments segments_current, segments_new;
  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      traj_msg,
      n_dof,
      now,
      segments_current);
}
#endif

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  // Initialize Orocos
  __os_init(argc, argv);

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  //RTT::Logger::log().setLogLevel(RTT::Logger::Info);

  // Import conman plugin
  //if(!RTT::ComponentLoader::Instance()->import("lcsr_controllers", "" )) {
    //std::cerr<<"Could not import lcsr_controllers package."<<std::endl;
    //return -1;
  //}
  
  return RUN_ALL_TESTS();
}

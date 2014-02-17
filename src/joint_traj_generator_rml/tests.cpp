
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

#include <conman/conman.h>
#include <conman/scheme.h>
#include <conman/hook.h>

#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <gtest/gtest.h>
#include <gmock/gmock.h>
using ::testing::ElementsAre;

class StaticTest : public ::testing::Test {
protected:
  StaticTest() { }
};

TEST_F(StaticTest, MsgConversion) 
{
  // create a sinusoidal trajectory
  size_t n_dof = 7;
  trajectory_msgs::JointTrajectory traj_msg;

  for(double t=0; t < 1.0; t+=0.2) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = t;
    point.positions.resize(n_dof);

    for(int n=0; n<n_dof; n++) {
      point.positions[n] = sin(n*2.0*M_PI*t);
      point.velocities[n] = cos(n*2.0*M_PI*t);
    }
    
    traj_msg.points.push_back(point);
  }

  // convert the trajectory message into a list of trajectory segments
  JointTrajGeneratorRML::TrajSegments segments;
  JointTrajGeneratorRML::TrajectoryMsgToSegments(
      traj_msg,
      n_dof,
      segments);

  EXPECT_EQ(segments.size(),traj_msg.points.size());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  // Initialize Orocos
  __os_init(argc, argv);

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);
  //RTT::Logger::log().setLogLevel(RTT::Logger::Info);

  // Import conman plugin
  //RTT::ComponentLoader::Instance()->import("conman", "" );

  return RUN_ALL_TESTS();
}

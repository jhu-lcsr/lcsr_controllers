cmake_minimum_required(VERSION 2.8.6)

project(lcsr_controllers)

find_package(catkin REQUIRED
  COMPONENTS 
  rtt_ros 
  rtt_roscomm
  rtt_rosparam
  rtt_ros_tools
  rtt_sensor_msgs
  kdl_urdf_tools
  kdl_conversions
  control_toolbox
  reflexxes_type2)

find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)

# Generate ROS messages
add_message_files(FILES
  SemiAbsoluteCalibrationState.msg)
add_service_files(FILES
  Calibrate.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package()

# Generate RTT typekit
ros_generate_rtt_typekit(${PROJECT_NAME})
ros_generate_rtt_service_proxies(${PROJECT_NAME})

orocos_use_package(conman)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS})
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS}
  ${USE_OROCOS_LIBRARY_DIRS})

orocos_component(${PROJECT_NAME}
  src/lcsr_controllers.cpp
  src/joint_pid_controller.cpp
  src/id_controller_kdl.cpp
  src/joint_traj_generator_kdl.cpp
  src/joint_traj_generator_rml.cpp
  src/semi_absolute_calibration_controller.cpp
  src/jt_nullspace_controller.cpp
  )

target_link_libraries(
  ${PROJECT_NAME}
  ${USE_OROCOS_LIBRARIES} 
  ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

orocos_generate_package(
  DEPENDS ${PROJECT_NAME}
  DEPENDS_TARGETS rtt_roscomm)

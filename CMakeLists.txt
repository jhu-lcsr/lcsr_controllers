cmake_minimum_required(VERSION 2.8.6)

project(lcsr_controllers)

find_package(catkin REQUIRED
  COMPONENTS 
  rtt_ros 
  rtt_roscomm
  rtt_ros_tools
  kdl_urdf_tools
  kdl_conversions
  std_msgs
  geometry_msgs
  control_msgs
  control_toolbox
  reflexxes_type2
  telemanip_msgs
  cmake_modules)

find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)

# Generate ROS messages
add_message_files(FILES
  SemiAbsoluteCalibrationState.msg)
add_service_files(FILES
  Calibrate.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package()

# Generate RTT typekit
ros_generate_rtt_typekit(${PROJECT_NAME})
ros_generate_rtt_service_proxies(${PROJECT_NAME})

orocos_use_package(conman)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS})
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${orocos_kdl_LIBRARY_DIRS})

add_library(lcsr_controllers_friction
  src/friction/joint_friction_compensator_hss.cpp)

orocos_component(${PROJECT_NAME}
  src/lcsr_controllers.cpp
  src/joint_pid_controller.cpp
  src/joint_setpoint.cpp
  src/id_controller_kdl.cpp
  src/ik_controller.cpp # new inverse kinematics controller
  src/joint_traj_generator_kdl.cpp
  src/joint_traj_generator_rml/joint_traj_generator_rml.cpp
  src/semi_absolute_calibration_controller.cpp
  )

set(COMPONENT_LIBS
  ${orocos_kdl_LIBRARIES}
  ${catkin_LIBRARIES})

target_link_libraries( ${PROJECT_NAME} ${COMPONENT_LIBS} lcsr_controllers_friction)

orocos_component(lcsr_controllers_jt_nullspace_controller src/jt_nullspace_controller.cpp)
orocos_component(lcsr_controllers_cartesian_logistic_servo src/cartesian_logistic_servo.cpp)
orocos_component(lcsr_controllers_coulomb_compensator src/coulomb_compensator.cpp)

target_link_libraries(lcsr_controllers_jt_nullspace_controller ${COMPONENT_LIBS} lcsr_controllers_friction)
target_link_libraries(lcsr_controllers_cartesian_logistic_servo ${COMPONENT_LIBS})
target_link_libraries(lcsr_controllers_coulomb_compensator ${COMPONENT_LIBS})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

orocos_generate_package(
  DEPENDS ${PROJECT_NAME}
  DEPENDS_TARGETS rtt_roscomm)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
  find_library(GMOCK_LIBRARY NAMES gmock)

  catkin_add_gtest(test_joint_trajectory_generator_rml src/joint_traj_generator_rml/tests.cpp)
  target_link_libraries(test_joint_trajectory_generator_rml 
    lcsr_controllers 
    ${catkin_LIBRARIES} 
    ${GMOCK_LIBRARY}
    ${USE_OROCOS_LIBRARIES})

endif()


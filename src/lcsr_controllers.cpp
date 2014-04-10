
#include <ocl/Component.hpp>

#include "id_controller_kdl.h"
#include "ik_controller.h"
#include "joint_pid_controller.h"
#include "joint_setpoint.h"
#include "joint_traj_generator_kdl.h"
#include "joint_traj_generator_rml/joint_traj_generator_rml.h"
#include "semi_absolute_calibration_controller.h"
#include "jt_nullspace_controller.h"

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(lcsr_controllers::IDControllerKDL)
ORO_LIST_COMPONENT_TYPE(lcsr_controllers::IKController)
ORO_LIST_COMPONENT_TYPE(lcsr_controllers::JointPIDController)
ORO_LIST_COMPONENT_TYPE(lcsr_controllers::JointSetpoint)
ORO_LIST_COMPONENT_TYPE(lcsr_controllers::JointTrajGeneratorKDL)
ORO_LIST_COMPONENT_TYPE(lcsr_controllers::JointTrajGeneratorRML)
ORO_LIST_COMPONENT_TYPE(lcsr_controllers::SemiAbsoluteCalibrationController)
ORO_LIST_COMPONENT_TYPE(lcsr_controllers::JTNullspaceController)


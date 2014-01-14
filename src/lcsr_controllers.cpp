
#include <ocl/Component.hpp>

#include "id_controller_kdl.h"
#include "joint_pid_controller.h"

ORO_CREATE_COMPONENT_LIBRARY()
ORO_LIST_COMPONENT_TYPE(lcsr_controllers::IDControllerKDL)
ORO_LIST_COMPONENT_TYPE(lcsr_controllers::JointPIDController)


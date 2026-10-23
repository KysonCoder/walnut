#ifndef __MICRO_STEP_HPP__
#define __MICRO_STEP_HPP__

// Micro-step payload carried by the shm ring queue _micro_list.
// Alias to the func-layer servo library definition (pure POD, layout pinned
// by static_asserts in servo_step_data.hpp).
#include "servo/servo_step_data.hpp"

using ServoStepData = servo::ServoStepData;

#endif

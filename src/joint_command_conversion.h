#pragma once

#include "abb_egm_rws_managers/utilities.h"

namespace abb
{
namespace robot
{
namespace detail
{
// Caller validates finite commands, position limits, and protobuf array sizes.
inline void copyJointCommand(const MotionData::Joint& joint, int index,
                             egm::wrapper::Joints* positions,
                             egm::wrapper::Joints* velocities)
{
  const double scale = joint.rotational ? Constants::RAD_TO_DEG : Constants::M_TO_MM;
  positions->set_values(index, joint.command.position * scale);
  // Preserve coordinated velocity feedforward, including arbitrarily slow motion.
  // A per-joint deadband can change the Cartesian direction while leaving the
  // position target unchanged. Always write zero too, to clear any prior value.
  velocities->set_values(index, joint.command.velocity * scale);
}
}
}
}

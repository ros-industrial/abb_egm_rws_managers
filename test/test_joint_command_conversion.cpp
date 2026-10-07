#include "joint_command_conversion.h"
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

static void require(bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}

int main()
{
  using namespace abb::robot;
  abb::egm::wrapper::Joints positions, velocities;
  for (int i = 0; i < 6; ++i)
  {
    positions.add_values(0.0);
    velocities.add_values(0.0);
  }

  // Actual first UNARC sample at +1.0012 s, episode 20261006_173026.
  // Regression: the old 0.5 deg/s deadband turns downward coordinated
  // velocity into upward feedforward by retaining only joint 3.
  const std::array<double, 6> speed_deg{{
    0.01832856843, 0.31111780177, -0.57942470554,
    0.01336781908, 0.26818641827, 0.01095208201}};
  const std::array<double, 6> jacobian_z{{
    0.0, -0.62802046598, -0.41491776488,
    0.00495258143, -0.24441832040, -0.00000044446}};
  double expected_z = 0.0, converted_z = 0.0, legacy_z = 0.0;
  MotionData::Joint joint{};
  joint.rotational = true;
  joint.command.position = 0.1;
  for (int i = 0; i < 6; ++i)
  {
    joint.command.velocity = speed_deg[i] / Constants::RAD_TO_DEG;
    detail::copyJointCommand(joint, i, &positions, &velocities);
    expected_z += jacobian_z[i] * joint.command.velocity;
    converted_z += jacobian_z[i] * velocities.values(i) / Constants::RAD_TO_DEG;
    if (std::abs(speed_deg[i]) > 0.5)
      legacy_z += jacobian_z[i] * joint.command.velocity;
    require(std::abs(positions.values(i) - 0.1 * Constants::RAD_TO_DEG) < 1e-12,
            "Position command changed");
  }
  require(expected_z < 0.0 && legacy_z > 0.0, "Fixture must reproduce direction reversal");
  require(std::abs(converted_z - expected_z) < 1e-12,
          "Conversion corrupts coordinated Cartesian velocity");

  // Reversal, threshold boundaries, tiny commands and stop must overwrite the
  // preceding command. Exercise the common robot/external joint conversion.
  for (bool rotational : {true, false})
  {
    joint.rotational = rotational;
    const double scale = rotational ? Constants::RAD_TO_DEG : Constants::M_TO_MM;
    for (double speed : {1.0, 0.500001, 0.5, 0.499999, 1e-8, 0.0, -1e-8, -0.5, -1.0, 0.0})
    {
      joint.command.velocity = speed / scale;
      detail::copyJointCommand(joint, 0, &positions, &velocities);
      require(std::abs(velocities.values(0) - speed) < 1e-12,
              "Small, reversed, or zero velocity lost");
      require(std::abs(positions.values(0) - joint.command.position * scale) < 1e-12,
              "Joint position units changed");
    }
  }
  std::cout << "PASS: recorded reversal, signed slow velocities, stop, and units\n";
}

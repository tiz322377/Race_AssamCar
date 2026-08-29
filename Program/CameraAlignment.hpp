#ifndef CAMERA_ALIGNMENT_HPP
#define CAMERA_ALIGNMENT_HPP

#include "ProgramContext.hpp"

namespace Program {

void alignRoughWithCamera(
    RobotHardware &_hardware,
    Rs485Chassis &_chassis,
    const AlignmentProfile &_profile);

} // namespace Program

#endif // CAMERA_ALIGNMENT_HPP

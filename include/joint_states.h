#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace mirrorme_robot_sdk {

struct JointState {
    uint32_t joint_id = 0;
    // Rotary/wheel joints use rad, rad/s and N*m, respectively.
    std::optional<float> position;
    std::optional<float> velocity;
    std::optional<float> torque;
};

struct JointStates {
    // Robot-relative monotonic snapshot time, not Unix epoch.
    uint64_t timestamp_ms = 0;
    // Age since the first fragment arrived, measured by the SDK's steady clock.
    uint64_t age_ms = 0;
    uint32_t sequence = 0;
    uint32_t topology_revision = 0;
    std::vector<JointState> joints;
};

}  // namespace mirrorme_robot_sdk

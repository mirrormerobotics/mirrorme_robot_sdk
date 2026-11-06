#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace mirrorme_robot_sdk {

struct JointTemperature {
    uint32_t joint_id = 0;
    std::optional<float> motor_celsius;
    std::optional<float> driver_celsius;
};

struct JointTemperatures {
    // Robot-relative monotonic sample time, not Unix epoch.
    uint64_t timestamp_ms = 0;
    // Age measured by the SDK's local steady clock at retrieval.
    uint64_t age_ms = 0;
    uint32_t sequence = 0;
    uint32_t topology_revision = 0;
    std::vector<JointTemperature> joints;
};

}  // namespace mirrorme_robot_sdk

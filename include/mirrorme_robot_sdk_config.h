#ifndef MIRRORME_ROBOT_SDK_CONFIG_H_
#define MIRRORME_ROBOT_SDK_CONFIG_H_

#include <cstdint>

namespace mirrorme_robot_sdk {

// Client-side UDP ports listened to by this SDK for robot state upload data.
constexpr uint16_t DEFAULT_CLIENT_ROBOT_STATE_UDP_PORT = 9873;
constexpr uint16_t DEFAULT_CLIENT_JOINT_STATE_UDP_PORT = 7895;

// Default robot server IP used by this SDK.
constexpr char DEFAULT_SERVER_IP[] = "10.21.20.1";

}  // namespace mirrorme_robot_sdk

#endif  // MIRRORME_ROBOT_SDK_CONFIG_H_

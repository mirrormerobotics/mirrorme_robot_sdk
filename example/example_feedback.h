#ifndef MIRRORME_ROBOT_SDK_EXAMPLE_FEEDBACK_H_
#define MIRRORME_ROBOT_SDK_EXAMPLE_FEEDBACK_H_

#include "request_robot_state.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>

namespace mirrorme_robot_sdk::example {

inline bool isBpw(const RequestRobotState& robot) {
    const auto model = robot.getRobotModelValue();
    return model && *model == RobotModel::BPW;
}

inline void printIdentityAvailability(const RequestRobotState& robot, bool show_identity = false) {
    const auto sn = robot.getRobotSerialNumberValue();
    const auto model = robot.getRobotModelValue();
    // Other examples keep relying on the SDK's identity log on connect().
    if (show_identity) {
        const char* model_name = "Unknown";
        if (model) {
            switch (*model) {
                case RobotModel::BPX: model_name = "BPX"; break;
                case RobotModel::BPXPro: model_name = "BPXPro"; break;
                case RobotModel::BPW: model_name = "BPW"; break;
                case RobotModel::Unknown: break;
            }
        }
        std::cout << "robot identity: serial_number="
                  << (sn ? (sn->empty() ? "<empty>" : *sn) : "<unavailable>") << " model=";
        if (model) std::cout << model_name << " (" << static_cast<unsigned>(*model) << ')';
        else std::cout << "<unavailable>";
        std::cout << std::endl;
    } else if (!sn || !model) {
        std::cout << "robot identity: <unavailable>" << std::endl;
    }
    if (isBpw(robot)) {
        std::cout << "BPW: basic state and motion (walk=flat, rough) supported; "
                     "fixed 12-joint state/temperature, leg odometry and JointLevelControl unsupported; dynamic joint state/temperature feedback is negotiated automatically."
                  << std::endl;
    }
}

inline std::string formatControlMode(const RequestRobotState& robot) {
    ControlMode mode;
    if (!robot.getControlMode(&mode)) return "<unavailable>";
    const char* name = "Unknown";
    switch (mode) {
        case ControlMode::RemoteControl: name = "RemoteControl"; break;
        case ControlMode::Navigator: name = "Navigator"; break;
        case ControlMode::Unknown: break;
    }
    return std::string(name) + " (" + std::to_string(static_cast<unsigned>(mode)) + ")";
}

inline std::string formatJointCount(const RequestRobotState& robot) {
    JointStates states;
    if (robot.getJointStates(&states)) return std::to_string(states.joints.size());
    JointTemperatures temperatures;
    return robot.getJointTemperatures(&temperatures) ? std::to_string(temperatures.joints.size()) : "unknown";
}

inline void printJointStates(const RequestRobotState& robot) {
    JointStates snapshot;
    if (!robot.getJointStates(&snapshot)) {
        std::cout << "  joint_states=<unavailable>" << std::endl;
        return;
    }
    std::cout << "  joint_states count=" << snapshot.joints.size()
              << " timestamp_ms=" << snapshot.timestamp_ms << " age_ms=" << snapshot.age_ms
              << " sequence=" << snapshot.sequence << " topology_revision=" << snapshot.topology_revision
              << std::endl;
    for (const auto& joint : snapshot.joints) {
        std::cout << "    joint_id=" << joint.joint_id << " position=";
        if (joint.position) std::cout << *joint.position; else std::cout << "<unavailable>";
        std::cout << " velocity=";
        if (joint.velocity) std::cout << *joint.velocity; else std::cout << "<unavailable>";
        std::cout << " torque=";
        if (joint.torque) std::cout << *joint.torque; else std::cout << "<unavailable>";
        std::cout << std::endl;
    }
}

inline void printJointTemperatures(const RequestRobotState& robot) {
    JointTemperatures snapshot;
    if (!robot.getJointTemperatures(&snapshot)) {
        std::cout << "  joint_temperatures=<unavailable>" << std::endl;
        return;
    }
    std::cout << "  joint_temperatures count=" << snapshot.joints.size()
              << " timestamp_ms=" << snapshot.timestamp_ms << " age_ms=" << snapshot.age_ms
              << " sequence=" << snapshot.sequence << " topology_revision=" << snapshot.topology_revision
              << std::endl;
    for (const auto& joint : snapshot.joints) {
        std::cout << "    joint_id=" << joint.joint_id << " motor_celsius=";
        if (joint.motor_celsius) std::cout << *joint.motor_celsius;
        else std::cout << "<unavailable>";
        std::cout << " driver_celsius=";
        if (joint.driver_celsius) std::cout << *joint.driver_celsius;
        else std::cout << "<unavailable>";
        std::cout << std::endl;
    }
}

inline void printFaultDetails(const RequestRobotState& robot) {
    RobotFaultState state;
    if (!robot.getFaultState(&state)) {
        std::cout << "  fault_details=<unavailable>" << std::endl;
        return;
    }
    std::cout << "  fault_details count=" << state.entry_count
              << " truncated=" << (state.truncated ? "true" : "false") << std::endl;
    for (size_t index = 0; index < state.entry_count && index < state.entries.size(); ++index) {
        std::cout << "    fault index=" << index << " code=0x" << std::hex << state.entries[index].code
                  << std::dec << " occurred_t_ms=" << state.entries[index].occurred_t_ms << std::endl;
    }
}

inline std::string formatPowerState(const RequestRobotState& robot) {
    const auto level = robot.getBatteryLevelValue();
    const auto current = robot.getBatteryCurrentValue();
    const auto in1 = robot.getChargerIn1Value();
    const auto in2 = robot.getChargerIn2Value();
    const auto hard_estop = robot.getHardEstopValue();
    const auto fault_state = robot.getFaultStateValue();
    std::ostringstream output;
    output << "battery_level=" << (level ? std::to_string(*level) + "%" : "<unavailable>")
           << " battery_current=";
    if (current) output << std::fixed << std::setprecision(3) << *current;
    else output << "<unavailable>";
    output << " charger_in1=" << (in1 ? std::to_string(*in1) : "<unavailable>")
           << " charger_in2=" << (in2 ? std::to_string(*in2) : "<unavailable>")
           << " hard_estop=" << (hard_estop ? std::to_string(*hard_estop) : "<unavailable>");
    if (fault_state) {
        output << " faults=" << fault_state->entry_count
               << (fault_state->truncated ? "+truncated" : "");
        if (fault_state->entry_count > 0) {
            output << " top=0x" << std::hex << fault_state->entries[0].code << std::dec;
        }
    } else {
        output << " faults=<unavailable>";
    }
    return output.str();
}

}  // namespace mirrorme_robot_sdk::example
#endif

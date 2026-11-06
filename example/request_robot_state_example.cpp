#include "request_robot_state.h"
#include "mirrorme_robot_sdk_version.h"
#include "example_options.h"
#include "example_feedback.h"

#include <chrono>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <optional>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int) {
    stop_requested = 1;
}

void printVersion() {
    std::cout << MIRRORME_ROBOT_SDK_PROJECT_NAME
              << " version=" << MIRRORME_ROBOT_SDK_PROJECT_VERSION << std::endl;
}

void printRobotVersion(const mirrorme_robot_sdk::RequestRobotState& robot_state) {
    uint16_t major = 0, minor = 0, patch = 0;
    uint32_t commit = 0, build = 0, build_time = 0;
    if (robot_state.getRobotVersion(&major, &minor, &patch, &commit, &build, &build_time)) {
        std::cout << "robot version (cached): " << major << '.' << minor << '.' << patch
                  << " commit=0x" << std::hex << commit << " build=" << std::dec << build
                  << 'T' << std::setw(6) << std::setfill('0') << build_time
                  << std::setfill(' ') << std::endl;
    } else {
        std::cout << "robot version (cached): <unavailable>" << std::endl;
    }
}

void printVector(const char* name, bool available, const std::vector<float>& values) {
    std::cout << "  " << name;
    if (!available) {
        std::cout << "=<unavailable>" << std::endl;
        return;
    }
    std::cout << " count=" << values.size() << " values=[";
    for (size_t index = 0; index < values.size(); ++index) {
        if (index) std::cout << ", ";
        std::cout << values[index];
    }
    std::cout << ']' << std::endl;
}

template <size_t N>
void printTuple(const char* name, bool available, const float (&values)[N]) {
    std::cout << "  " << name << '=';
    if (!available) {
        std::cout << "<unavailable>" << std::endl;
        return;
    }
    std::cout << '(';
    for (size_t index = 0; index < N; ++index) {
        if (index) std::cout << ", ";
        std::cout << values[index];
    }
    std::cout << ')' << std::endl;
}

std::string integerText(bool available, int64_t value) {
    return available ? std::to_string(value) : "<unavailable>";
}

void printMotionState(const mirrorme_robot_sdk::RequestRobotState& robot_state) {
    uint8_t state = 0, gait = 0, sub_gait = 0, last_state = 0, last_gait = 0;
    const bool has_state = robot_state.getCurrentMotionState(&state);
    const bool has_gait = robot_state.getCurrentGait(&gait);
    const bool has_sub_gait = robot_state.getSubGait(&sub_gait);
    const bool has_last_state = robot_state.getLastMotionState(&last_state);
    const bool has_last_gait = robot_state.getLastGait(&last_gait);
    std::cout << "  motion_state=" << integerText(has_state, state)
              << " gait=" << integerText(has_gait, gait)
              << " sub_gait=" << integerText(has_sub_gait, static_cast<int8_t>(sub_gait))
              << " last_state=" << integerText(has_last_state, last_state)
              << " last_gait=" << integerText(has_last_gait, last_gait) << std::endl;
    float max_velocity[3]{};
    const bool has_max_velocity = robot_state.getMaxVelocity(max_velocity);
    printTuple("max_vel", has_max_velocity, max_velocity);
}

void printTimestamps(const mirrorme_robot_sdk::RequestRobotState& robot_state, bool bpw) {
    uint32_t joint = 0, imu = 0, odometry = 0, motion = 0, battery = 0, fault = 0;
    const bool has_joint = robot_state.getJointStateTimestamp(&joint);
    const bool has_imu = robot_state.getImuTimestamp(&imu);
    const bool has_odometry = !bpw && robot_state.getOdometryTimestamp(&odometry);
    const bool has_motion = robot_state.getMotionStateTimestamp(&motion);
    const bool has_battery = robot_state.getBatteryTimestamp(&battery);
    const bool has_fault = robot_state.getFaultTimestamp(&fault);
    std::cout << "  timestamps_ms joint=" << integerText(has_joint, joint)
              << " imu=" << integerText(has_imu, imu)
              << " odometry=" << (bpw ? "<unsupported>" : integerText(has_odometry, odometry))
              << " motion=" << integerText(has_motion, motion)
              << " battery=" << integerText(has_battery, battery)
              << " fault=" << integerText(has_fault, fault) << std::endl;
}

void printLegOdometry(const mirrorme_robot_sdk::RequestRobotState& robot_state, bool bpw) {
    if (bpw) {
        std::cout << "  leg_odom=<unsupported>" << std::endl;
        return;
    }
    mirrorme_robot_sdk::LegOdom odometry{};
    if (!robot_state.getLegOdom(&odometry)) {
        std::cout << "  leg_odom=<unavailable>" << std::endl;
        return;
    }
    std::cout << "  leg_odom.position=(" << odometry.position[0] << ", " << odometry.position[1]
              << ", " << odometry.position[2] << ") velocity_body=(" << odometry.velocity_body[0]
              << ", " << odometry.velocity_body[1] << ", " << odometry.velocity_body[2]
              << ") orientation=(" << odometry.orientation[0] << ", " << odometry.orientation[1]
              << ", " << odometry.orientation[2] << ", " << odometry.orientation[3]
              << ") angular_velocity=(" << odometry.angular_velocity[0] << ", "
              << odometry.angular_velocity[1] << ", " << odometry.angular_velocity[2] << ')' << std::endl;
}

void printFrameTime() {
    const auto now = std::chrono::system_clock::now();
    const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - seconds).count();
    const std::time_t timestamp = std::chrono::system_clock::to_time_t(seconds);
    std::tm local_time{};
#ifdef _WIN32
    localtime_s(&local_time, &timestamp);
#else
    localtime_r(&timestamp, &local_time);
#endif
    std::cout << "robot state: " << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S")
              << '.' << std::setw(3) << std::setfill('0') << milliseconds << std::setfill(' ') << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
    mirrorme_robot_sdk::example::Options options;
    if (!mirrorme_robot_sdk::example::parseOptions(argc, argv, false, &options)) return 1;
    if (options.help_requested) return 0;

    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    std::cout << std::defaultfloat << std::setprecision(6);
    printVersion();

    mirrorme_robot_sdk::RequestRobotState robot_state;
    robot_state.setRobotIp(options.robot_ip.c_str());
    robot_state.setRobotStateUploadPort(options.robot_state_port);
    robot_state.setTcpLocalPort(options.tcp_local_port);
    robot_state.setRobotStateUploadRate(options.state_rate_hz);
    std::cout << "robot_ip=" << options.robot_ip << " state_port=" << options.robot_state_port
              << " tcp_local_port=" << options.tcp_local_port << " state_rate=" << options.state_rate_hz << std::endl;
    std::cout << "connection: connecting" << std::endl;
    if (!robot_state.connect()) {
        std::cerr << "failed to connect request robot state" << std::endl;
        robot_state.disconnect();
        std::cout << "connection: stopped" << std::endl;
        return 1;
    }

    bool was_connected = false;
    bool ever_connected = false;
    std::optional<mirrorme_robot_sdk::RobotModel> last_model;
    while (!stop_requested) {
        const bool connected = robot_state.isConnected();
        if (connected != was_connected) {
            if (connected) {
                std::cout << (ever_connected ? "connection: reconnected" : "connection: connected") << std::endl;
                ever_connected = true;
                printRobotVersion(robot_state);
                mirrorme_robot_sdk::example::printIdentityAvailability(robot_state, true);
            } else {
                std::cout << "connection: disconnected; retrying" << std::endl;
            }
            was_connected = connected;
        }
        printFrameTime();
        std::cout << "  connected=" << (connected ? "true" : "false") << std::endl;
        if (!connected) {
            std::cout << "  telemetry=<unavailable: disconnected>" << std::endl;
        } else {
            mirrorme_robot_sdk::RobotModel model;
            if (robot_state.getRobotModel(&model)) last_model = model;
            const bool bpw = last_model && *last_model == mirrorme_robot_sdk::RobotModel::BPW;
            std::cout << "  control_mode=" << mirrorme_robot_sdk::example::formatControlMode(robot_state) << std::endl;
            mirrorme_robot_sdk::example::printJointStates(robot_state);
            std::vector<float> position, velocity, torque;
            const bool has_position = robot_state.getJointPosition(position);
            const bool has_velocity = robot_state.getJointVelocity(velocity);
            const bool has_torque = robot_state.getJointTorque(torque);
            printVector("joint_pos", has_position, position);
            printVector("joint_vel", has_velocity, velocity);
            printVector("joint_tau", has_torque, torque);

            float rpy[3]{}, quaternion[4]{}, acceleration[3]{}, omega[3]{};
            const bool has_rpy = robot_state.getImuRpy(rpy);
            const bool has_quaternion = robot_state.getImuQuat(quaternion);
            const bool has_acceleration = robot_state.getImuAcc(acceleration);
            const bool has_omega = robot_state.getImuOmega(omega);
            printTuple("rpy", has_rpy, rpy);
            printTuple("quat", has_quaternion, quaternion);
            printTuple("acc", has_acceleration, acceleration);
            printTuple("omega", has_omega, omega);
            printLegOdometry(robot_state, bpw);
            printMotionState(robot_state);
            printTimestamps(robot_state, bpw);

            mirrorme_robot_sdk::example::printJointTemperatures(robot_state);
            std::vector<float> motor_temperature, driver_temperature;
            const bool has_motor_temperature = robot_state.getMotorTemperature(motor_temperature);
            const bool has_driver_temperature = robot_state.getDriverTemperature(driver_temperature);
            printVector("motor_temperature", has_motor_temperature, motor_temperature);
            printVector("driver_temperature", has_driver_temperature, driver_temperature);
            std::cout << "  " << mirrorme_robot_sdk::example::formatPowerState(robot_state) << std::endl;
            mirrorme_robot_sdk::example::printFaultDetails(robot_state);
        }
        std::cout << std::endl;
        if (!stop_requested) std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    robot_state.disconnect();
    std::cout << "connection: stopped" << std::endl;
    return 0;
}

#ifndef MIRRORME_ROBOT_SDK_REQUEST_ROBOT_STATE_H_
#define MIRRORME_ROBOT_SDK_REQUEST_ROBOT_STATE_H_

#include "mirrorme_robot_sdk_config.h"
#include "fault_code.h"
#include "motion_types.h"
#include "joint_temperatures.h"
#include "joint_states.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace mirrorme_robot_sdk {

class RequestRobotState {
public:
    RequestRobotState();
    virtual ~RequestRobotState();

    // Queries SN/model over TCP and logs the result before starting the
    // background subscription. Unsupported queries time out without failing
    // connect(). Automatic reconnects refresh identity over TCP as well.
    virtual bool connect();
    virtual void disconnect();
    bool isConnected() const;
    void setRobotStateUploadPort(uint16_t);
    virtual void setJointStateUploadPort(uint16_t);
    // Request the robot-to-SDK ordinary UDP state upload rate. Call before
    // connect(); valid range is 1..1000 Hz and the robot falls back to 100 Hz
    // for invalid values. Per-state-group native rate caps still apply.
    void setRobotStateUploadRate(uint16_t rate_hz);
    void setTcpLocalPort(uint16_t port);
    void setSessionId(uint16_t);
    virtual void setRobotIp(const char*);

    // Query the robot (motion host) build version. Uses an independent TCP
    // connection, so it may be called before connect(). Returns false when the
    // robot does not support version query. All output pointers are optional.
    bool queryRobotVersion(uint16_t* major, uint16_t* minor, uint16_t* patch,
                           uint32_t* commit = nullptr, uint32_t* build_date = nullptr,
                           uint32_t* build_time = nullptr);

    // Read the robot version cached during connect() (no network I/O). Returns
    // false if connect() has not yet succeeded or the robot did not support
    // version query. Use queryRobotVersion() for a fresh on-demand query.
    bool getRobotVersion(uint16_t* major, uint16_t* minor, uint16_t* patch,
                         uint32_t* commit = nullptr, uint32_t* build_date = nullptr,
                         uint32_t* build_time = nullptr) const;

    // Current velocity-control source from TCP subscription feedback.
    // False/nullopt until supported feedback arrives; cleared on disconnect.
    bool getControlMode(ControlMode* mode) const;
    std::optional<ControlMode> getControlModeValue() const;

    // Identity cached by the synchronous TCP query during connect/reconnect.
    // False if the query fails or the robot does not support it.
    // A received empty SN means the robot could not read its whole-robot SN.
    bool getRobotSerialNumber(std::string* serial_number) const;
    bool getRobotModel(RobotModel* model) const;
    std::optional<std::string> getRobotSerialNumberValue() const;
    std::optional<RobotModel> getRobotModelValue() const;

    // Fixed 12-joint position/velocity/torque and temperature APIs (including
    // their non-template Array variants)
    // are unsupported on BPW; no partial 16-to-12 joint view is returned.
    [[deprecated("Fixed 12-joint API; use getJointPosition(std::vector<float>&) or getJointStates().")]]
    bool getJointPosition(float joint_pos[12]) const;
    [[deprecated("Fixed 12-joint API; use getJointVelocity(std::vector<float>&) or getJointStates().")]]
    bool getJointVelocity(float joint_vel[12]) const;
    [[deprecated("Fixed 12-joint API; use getJointTorque(std::vector<float>&) or getJointStates().")]]
    bool getJointTorque(float joint_tau[12]) const;

    // Complete same-frame states. Joint IDs identify samples; do not assume a
    // fixed axis count or layout. Data at least 1 second old is unavailable.
    // Negotiation is automatic. Legacy fallback is restricted to identified
    // BPX/BPX-Pro peers that do not advertise the joint-state extension.
    bool getJointStates(JointStates* states) const;
    std::optional<JointStates> getJointStatesValue() const;
    // Each dimension must be present for every joint. False leaves out unchanged.
    bool getJointPosition(std::vector<float>& joint_pos) const;
    bool getJointVelocity(std::vector<float>& joint_vel) const;
    bool getJointTorque(std::vector<float>& joint_tau) const;

    bool getImuRpy(float rpy[3]) const;    // ZYX Euler angles: roll, pitch, yaw.
    bool getImuQuat(float quat[4]) const;  // Quaternion order: x, y, z, w.
    bool getImuAcc(float acc[3]) const;
    bool getImuOmega(float omega[3]) const;

    // BPW leg odometry and its timestamp are not implemented.
    bool getLegOdom(LegOdom* leg_odom) const;

    // Complete same-frame temperatures. Joint IDs identify samples; do not
    // assume a fixed axis count or layout. Negotiation is automatic.
    // Data at least 3 seconds old is unavailable. Legacy fallback is restricted to
    // positively identified BPX/BPX-Pro peers without capability negotiation.
    bool getJointTemperatures(JointTemperatures* temperatures) const;
    std::optional<JointTemperatures> getJointTemperaturesValue() const;
    // Dynamic size; false leaves the caller's vector unchanged. All requested
    // temperatures must be present; inspect the snapshot for partial sensors.
    bool getMotorTemperature(std::vector<float>& motor_temperature) const;
    bool getDriverTemperature(std::vector<float>& driver_temperature) const;

    [[deprecated("Fixed 12-joint API; use getMotorTemperature(std::vector<float>&) or getJointTemperatures().")]]
    bool getMotorTemperature(float motor_temperature[12]) const;
    [[deprecated("Fixed 12-joint API; use getDriverTemperature(std::vector<float>&) or getJointTemperatures().")]]
    bool getDriverTemperature(float driver_temperature[12]) const;
    bool getCurrentMotionState(uint8_t* current_state) const;
    // On supporting firmware, current gait/sub-gait retain the pre-lie
    // selection during SitDown/WaitingForStand, matching robot_state.
    // StandingUp and motion states report the actual controller gait.
    bool getCurrentGait(uint8_t* current_gait) const;
    bool getLastMotionState(uint8_t* last_state) const;
    bool getLastGait(uint8_t* last_gait) const;
    bool getSubGait(uint8_t* sub_gait) const;
    bool getCurrentMotionState(MotionState* current_state) const;
    bool getCurrentGait(MotionGait* current_gait) const;
    bool getLastMotionState(MotionState* last_state) const;
    bool getLastGait(MotionGait* last_gait) const;
    bool getMaxVelocity(float max_vel[3]) const;

    bool getBatteryLevel(uint8_t* battery_level) const;
    bool getBatteryCurrent(float* battery_current) const;
    // Decode 1 Hz battery flags to 0 (removed) / 1 (inserted).
    // Requires robot firmware using the same bitmask charger encoding.
    bool getChargerIn1(uint8_t* charger_in1) const;
    bool getChargerIn2(uint8_t* charger_in2) const;
    // Hard estop switch state from the same 1 Hz flags: 1 = pressed (all
    // joints unpowered), 0 = released. False until a 1 Hz packet arrives.
    bool getHardEstop(uint8_t* hard_estop) const;
    // Full active-fault snapshot from the dedicated 1 Hz fault channel
    // (head=0x00F0): up to kMaxFaultEntries entries sorted by severity
    // descending, entries[0] being the current top-priority fault. Requires
    // robot firmware that publishes the channel; the robot keeps sending
    // count=0 snapshots while fault-free, so false/nullopt only means no
    // fault packet has been received yet (e.g. an older robot).
    bool getFaultState(RobotFaultState* fault_state) const;
    // Low 32 bits of the complete joint snapshot's robot timestamp for N-axis
    // telemetry. Use JointStates::timestamp_ms for the full 64-bit value.
    // Legacy BPX/BPX-Pro peers retain the ordinary 0x1000 channel timestamp.
    bool getJointStateTimestamp(uint32_t* time_ms) const;
    bool getImuTimestamp(uint32_t* time_ms) const;
    bool getOdometryTimestamp(uint32_t* time_ms) const;
    bool getMotionStateTimestamp(uint32_t* time_ms) const;
    bool getBatteryTimestamp(uint32_t* time_ms) const;
    bool getFaultTimestamp(uint32_t* time_ms) const;

    [[deprecated("Fixed 12-joint API; use getJointPositionArray<N>() or getJointStatesValue().")]]
    std::optional<std::array<float, 12>> getJointPositionArray() const;
    [[deprecated("Fixed 12-joint API; use getJointVelocityArray<N>() or getJointStatesValue().")]]
    std::optional<std::array<float, 12>> getJointVelocityArray() const;
    [[deprecated("Fixed 12-joint API; use getJointTorqueArray<N>() or getJointStatesValue().")]]
    std::optional<std::array<float, 12>> getJointTorqueArray() const;

    // Explicit-size views of dynamic state: request e.g. <16> for BPW. A size
    // mismatch or any missing value returns nullopt without truncating joints.
    template <std::size_t N>
    std::optional<std::array<float, N>> getJointPositionArray() const {
        std::vector<float> values;
        if (!getJointPosition(values) || values.size() != N) return std::nullopt;
        std::array<float, N> result{};
        for (std::size_t index = 0; index < N; ++index) result[index] = values[index];
        return result;
    }
    template <std::size_t N>
    std::optional<std::array<float, N>> getJointVelocityArray() const {
        std::vector<float> values;
        if (!getJointVelocity(values) || values.size() != N) return std::nullopt;
        std::array<float, N> result{};
        for (std::size_t index = 0; index < N; ++index) result[index] = values[index];
        return result;
    }
    template <std::size_t N>
    std::optional<std::array<float, N>> getJointTorqueArray() const {
        std::vector<float> values;
        if (!getJointTorque(values) || values.size() != N) return std::nullopt;
        std::array<float, N> result{};
        for (std::size_t index = 0; index < N; ++index) result[index] = values[index];
        return result;
    }

    std::optional<std::array<float, 3>> getImuRpyArray() const;    // ZYX Euler angles: roll, pitch, yaw.
    std::optional<std::array<float, 4>> getImuQuatArray() const;   // Quaternion order: x, y, z, w.
    std::optional<std::array<float, 3>> getImuAccArray() const;
    std::optional<std::array<float, 3>> getImuOmegaArray() const;

    std::optional<LegOdom> getLegOdomValue() const;

    [[deprecated("Fixed 12-joint API; use getMotorTemperature(std::vector<float>&) or getJointTemperaturesValue().")]]
    std::optional<std::array<float, 12>> getMotorTemperatureArray() const;
    [[deprecated("Fixed 12-joint API; use getDriverTemperature(std::vector<float>&) or getJointTemperaturesValue().")]]
    std::optional<std::array<float, 12>> getDriverTemperatureArray() const;
    std::optional<uint8_t> getCurrentMotionStateValue() const;
    std::optional<uint8_t> getCurrentGaitValue() const;
    std::optional<uint8_t> getLastMotionStateValue() const;
    std::optional<uint8_t> getLastGaitValue() const;
    std::optional<uint8_t> getSubGaitValue() const;
    std::optional<MotionState> getCurrentMotionStateEnum() const;
    std::optional<MotionGait> getCurrentGaitEnum() const;
    std::optional<MotionState> getLastMotionStateEnum() const;
    std::optional<MotionGait> getLastGaitEnum() const;
    std::optional<std::array<float, 3>> getMaxVelocityArray() const;

    std::optional<uint8_t> getBatteryLevelValue() const;
    std::optional<float> getBatteryCurrentValue() const;
    std::optional<uint8_t> getChargerIn1Value() const;
    std::optional<uint8_t> getChargerIn2Value() const;
    std::optional<uint8_t> getHardEstopValue() const;
    std::optional<RobotFaultState> getFaultStateValue() const;
    std::optional<uint32_t> getJointStateTimestampValue() const;
    std::optional<uint32_t> getImuTimestampValue() const;
    std::optional<uint32_t> getOdometryTimestampValue() const;
    std::optional<uint32_t> getMotionStateTimestampValue() const;
    std::optional<uint32_t> getBatteryTimestampValue() const;
    std::optional<uint32_t> getFaultTimestampValue() const;

protected:
    virtual uint8_t hostServerMode() const;
    virtual void onStateConnectionChanged(bool connected);
    // Called after an accepted subscription, before publishing connected.
    // A local resource failure returns false and retries the whole connection.
    virtual bool prepareStateConnection();
    const char* robotIp() const;
    // Last confirmed model survives disconnect for unsupported-operation
    // checks; changing the robot IP resets it. Public identity remains live.
    bool isBpwRobot() const;
    bool rejectBpwOperation(const char* operation) const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mirrorme_robot_sdk

#endif  // MIRRORME_ROBOT_SDK_REQUEST_ROBOT_STATE_H_

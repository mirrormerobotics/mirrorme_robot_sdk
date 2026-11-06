#include "motion_level_control.h"
#include "mirrorme_robot_sdk_version.h"
#include "example_options.h"
#include "example_feedback.h"

#include <chrono>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
using mirrorme_robot_sdk::MotionLevelControl;
using mirrorme_robot_sdk::MotionState;
using mirrorme_robot_sdk::RobotModel;

volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int) { stop_requested = 1; }

enum class DemoPhase { kConnecting, kPrepare, kStandUp, kWalk, kYaw, kZero, kSitDown, kDone };

void printVersion() {
    std::cout << MIRRORME_ROBOT_SDK_PROJECT_NAME
              << " version=" << MIRRORME_ROBOT_SDK_PROJECT_VERSION << std::endl;
}

void printRobotVersion(const mirrorme_robot_sdk::RequestRobotState& robot) {
    uint16_t major = 0, minor = 0, patch = 0;
    uint32_t commit = 0, build = 0, build_time = 0;
    if (robot.getRobotVersion(&major, &minor, &patch, &commit, &build, &build_time)) {
        std::cout << "robot version (cached): " << major << '.' << minor << '.' << patch
                  << " commit=0x" << std::hex << commit << std::dec << " build=" << build
                  << 'T' << std::setw(6) << std::setfill('0') << build_time << std::setfill(' ')
                  << std::endl;
    } else {
        std::cout << "robot version (cached): <unavailable>" << std::endl;
    }
}

struct MotionFeedback {
    uint32_t timestamp = 0;
    uint8_t state = 0;
    uint8_t gait = 0;
    bool has_timestamp = false;
    bool has_state = false;
    bool has_gait = false;

    bool complete() const { return has_timestamp && has_state && has_gait; }
};

MotionFeedback readFeedback(const MotionLevelControl& motion) {
    MotionFeedback feedback;
    feedback.has_timestamp = motion.getMotionStateTimestamp(&feedback.timestamp);
    feedback.has_state = motion.getCurrentMotionState(&feedback.state);
    feedback.has_gait = motion.getCurrentGait(&feedback.gait);
    return feedback;
}

void printStatus(const MotionLevelControl& motion, bool connected, const MotionFeedback& feedback) {
    if (!connected) {
        std::cout << "connected=false motion_state=<unavailable> gait=<unavailable>" << std::endl;
        return;
    }
    std::cout << "motion_state=" << (feedback.has_state ? std::to_string(feedback.state) : "<unavailable>")
              << " gait=" << (feedback.has_gait ? std::to_string(feedback.gait) : "<unavailable>")
              << " control_mode=" << mirrorme_robot_sdk::example::formatControlMode(motion)
              << ' ' << mirrorme_robot_sdk::example::formatPowerState(motion) << " max_vel=";
    float max_velocity[3]{};
    if (motion.getMaxVelocity(max_velocity)) {
        std::cout << '(' << max_velocity[0] << ", " << max_velocity[1] << ", " << max_velocity[2] << ')';
    } else {
        std::cout << "<unavailable>";
    }
    std::cout << " motion_timestamp_ms="
              << (feedback.has_timestamp ? std::to_string(feedback.timestamp) : "<unavailable>") << std::endl;
}

void announcePhase(DemoPhase phase) {
    switch (phase) {
        case DemoPhase::kPrepare:
            std::cout << "phase: prepare" << std::endl;
            break;
        case DemoPhase::kStandUp:
            std::cout << "phase: stand-up\nsend stand-up command" << std::endl;
            break;
        case DemoPhase::kWalk:
            std::cout << "phase: walk\nselect walk gait; waiting for fresh feedback" << std::endl;
            break;
        case DemoPhase::kYaw:
            std::cout << "phase: yaw\nsend yaw velocity command: 1.0 rad/s for 3 seconds" << std::endl;
            break;
        case DemoPhase::kZero:
            std::cout << "phase: zero-velocity\nsend zero velocity command for 2 seconds" << std::endl;
            break;
        case DemoPhase::kSitDown:
            std::cout << "phase: sit-down\nsend sit-down command; waiting for fresh feedback" << std::endl;
            break;
        default: break;
    }
}
}  // namespace

int main(int argc, char** argv) {
    mirrorme_robot_sdk::example::Options options;
    if (!mirrorme_robot_sdk::example::parseOptions(argc, argv, false, &options)) return 1;
    if (options.help_requested) return 0;

    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    printVersion();
    MotionLevelControl motion;
    motion.setRobotIp(options.robot_ip.c_str());
    motion.setRobotStateUploadPort(options.robot_state_port);
    motion.setTcpLocalPort(options.tcp_local_port);
    motion.setRobotStateUploadRate(options.state_rate_hz);
    motion.setMotionCommandRate(50);
    std::cout << "robot_ip=" << options.robot_ip << " state_port=" << options.robot_state_port
              << " tcp_local_port=" << options.tcp_local_port << " state_rate=" << options.state_rate_hz << std::endl;

    DemoPhase phase = DemoPhase::kConnecting;
    const auto connection_start = Clock::now();
    auto phase_start = connection_start;
    auto next_status = connection_start;
    auto last_feedback_change = connection_start;
    std::optional<Clock::time_point> prepare_command_time;
    std::optional<Clock::time_point> yaw_start;
    RobotModel model = RobotModel::Unknown;
    uint32_t last_feedback_timestamp = 0;
    uint32_t phase_baseline = 0;
    bool saw_connection = false;
    bool ready = false;
    bool phase_command_sent = false;
    int exit_code = 0;
    const auto fail = [&](const char* message) {
        std::cerr << message << std::endl;
        exit_code = 1;
    };
    const auto enter_phase = [&](DemoPhase next, uint32_t timestamp) {
        phase = next;
        phase_start = Clock::now();
        phase_baseline = timestamp;
        phase_command_sent = false;
        announcePhase(next);
    };
    const auto capture_command_baseline = [&] {
        // Record after the first successful command, and never accept feedback
        // from that same loop iteration as command-completion evidence.
        uint32_t timestamp = 0;
        if (motion.getMotionStateTimestamp(&timestamp)) phase_baseline = timestamp;
        phase_command_sent = true;
    };

    std::cout << "connection: connecting" << std::endl;
    const bool started = motion.connect();
    if (!started && !stop_requested) fail("failed to connect motion level control");
    while (started && !stop_requested && exit_code == 0 && phase != DemoPhase::kDone) {
        const auto now = Clock::now();
        const bool connected = motion.isConnected();
        const auto feedback = readFeedback(motion);
        if (connected) saw_connection = true;
        else if (saw_connection) {
            fail("connection lost");
            break;
        }
        if (now >= next_status) {
            printStatus(motion, connected, feedback);
            next_status = now + 200ms;
        }
        bool motion_progressed = false;
        if (ready) {
            if (feedback.has_timestamp && feedback.timestamp != last_feedback_timestamp) {
                motion_progressed = true;
                last_feedback_timestamp = feedback.timestamp;
                last_feedback_change = now;
            }
            if (now - last_feedback_change >= 1s) {
                fail("motion feedback timeout");
                break;
            }
        }
        const bool fresh = motion_progressed && phase_command_sent && feedback.complete() &&
                           feedback.timestamp != phase_baseline;
        switch (phase) {
            case DemoPhase::kConnecting: {
                if (now - connection_start >= 10s) {
                    fail("connection timeout");
                    break;
                }
                RobotModel observed = RobotModel::Unknown;
                const bool known_model = motion.getRobotModel(&observed) &&
                    (observed == RobotModel::BPX || observed == RobotModel::BPXPro || observed == RobotModel::BPW);
                if (connected && known_model && feedback.complete()) {
                    model = observed;
                    ready = true;
                    last_feedback_timestamp = feedback.timestamp;
                    last_feedback_change = now;
                    motion.setVelocityControlFlag(true);
                    std::cout << "connection: connected" << std::endl;
                    printRobotVersion(motion);
                    mirrorme_robot_sdk::example::printIdentityAvailability(motion, true);
                    std::cout << "motion level control running" << std::endl;
                    enter_phase(DemoPhase::kPrepare, feedback.timestamp);
                }
                break;
            }
            case DemoPhase::kPrepare:
                if (!prepare_command_time && now - phase_start >= 20ms) {
                    // Allow the sender's first 50 Hz zero baseline to run before
                    // the one-shot calibration setter, which has no return value.
                    if (model == RobotModel::BPW) {
                        std::cout << "BPW: skip zero-position command" << std::endl;
                    } else {
                        motion.setZeroPositionsFlag();
                        std::cout << "send zero-position command" << std::endl;
                    }
                    std::cout << "wait 1 second before sending commands" << std::endl;
                    prepare_command_time = Clock::now();
                }
                if (prepare_command_time && now - *prepare_command_time >= 1s)
                    enter_phase(DemoPhase::kStandUp, feedback.timestamp);
                break;
            case DemoPhase::kStandUp:
                if (now - phase_start >= 10s) fail("stand-up timeout");
                else if (!motion.setStandUp()) fail("failed to send stand-up command");
                else if (!phase_command_sent) capture_command_baseline();
                else if (fresh && feedback.state == static_cast<uint8_t>(MotionState::Motion))
                    enter_phase(DemoPhase::kWalk, feedback.timestamp);
                break;
            case DemoPhase::kWalk:
                if (now - phase_start >= 10s) {
                    fail("walk gait timeout");
                    break;
                }
                motion.setWalk();
                if (!motion.setVelocity(0.f, 0.f, 0.f)) fail("failed to send walk zero velocity command");
                else if (!phase_command_sent) capture_command_baseline();
                else if (fresh && feedback.state == static_cast<uint8_t>(MotionState::Motion) &&
                         feedback.gait == static_cast<uint8_t>(mirrorme_robot_sdk::MotionGait::Walk))
                    enter_phase(DemoPhase::kYaw, feedback.timestamp);
                break;
            case DemoPhase::kYaw:
                if (yaw_start && now - *yaw_start >= 3s) {
                    if (!motion.setVelocity(0.f, 0.f, 0.f)) fail("failed to send zero velocity command");
                    else enter_phase(DemoPhase::kZero, feedback.timestamp);
                } else if (!motion.setVelocity(0.f, 0.f, 1.0f)) {
                    fail("failed to send yaw velocity command");
                } else if (!yaw_start) {
                    yaw_start = Clock::now();
                }
                break;
            case DemoPhase::kZero:
                if (!motion.setVelocity(0.f, 0.f, 0.f)) fail("failed to send zero velocity command");
                else if (now - phase_start >= 2s) enter_phase(DemoPhase::kSitDown, feedback.timestamp);
                break;
            case DemoPhase::kSitDown:
                if (now - phase_start >= 10s) fail("sit-down timeout");
                else if (!motion.setSitDown()) fail("failed to send sit-down command");
                else if (!phase_command_sent) capture_command_baseline();
                else if (fresh && feedback.state == static_cast<uint8_t>(MotionState::LyingDown)) {
                    phase = DemoPhase::kDone;
                    std::cout << "motion demo: completed" << std::endl;
                }
                break;
            case DemoPhase::kDone: break;
        }
        if (!stop_requested && exit_code == 0 && phase != DemoPhase::kDone) std::this_thread::sleep_for(20ms);
    }
    // Velocity cleanup is appropriate only while running a locomotion phase.
    // Never turn a successful sit-down into another Motion/Walk request.
    if ((phase == DemoPhase::kWalk || phase == DemoPhase::kYaw || phase == DemoPhase::kZero) && motion.isConnected())
        motion.setVelocity(0.f, 0.f, 0.f);
    motion.disconnect();
    std::cout << "connection: stopped" << std::endl;
    return stop_requested ? 0 : exit_code;
}

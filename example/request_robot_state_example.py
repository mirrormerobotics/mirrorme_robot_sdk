import signal
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mirrorme_robot_sdk
from example_options import parse_options
from example_feedback import (format_power_state, format_control_mode, print_identity_availability,
                              format_optional, format_tuple, print_joint_list,
                              print_joint_temperatures, print_joint_states, print_fault_details)


def print_version():
    print(f"mirrorme_robot_sdk version={getattr(mirrorme_robot_sdk, '__version__', 'unknown')}")


def print_robot_version(robot_state):
    robot_version = robot_state.getRobotVersion()
    if robot_version is None:
        print("robot version (cached): <unavailable>")
        return

    major, minor, patch, commit, build_date, build_time = robot_version
    print(
        f"robot version (cached): "
        f"{major}.{minor}.{patch} "
        f"commit=0x{commit:x} "
        f"build={build_date}T{build_time:06d}"
    )


def print_telemetry(robot_state, bpw):
    print(f"  control_mode={format_control_mode(robot_state)}")
    # Each convenience getter has its own validity result.
    print_joint_list("joint_pos", robot_state.getJointPosition())
    print_joint_list("joint_vel", robot_state.getJointVelocity())
    print_joint_list("joint_tau", robot_state.getJointTorque())
    print_joint_states(robot_state)

    print(f"  rpy={format_tuple(robot_state.getImuRpy())}")
    print(f"  quat={format_tuple(robot_state.getImuQuat())}")
    print(f"  acc={format_tuple(robot_state.getImuAcc())}")
    print(f"  omega={format_tuple(robot_state.getImuOmega())}")

    state = robot_state.getCurrentMotionState()
    gait = robot_state.getCurrentGait()
    sub_gait = robot_state.getSubGait()
    last_state = robot_state.getLastMotionState()
    last_gait = robot_state.getLastGait()
    if sub_gait is not None:
        sub_gait = (int(sub_gait) + 128) % 256 - 128
    print(f"  motion_state={format_optional(state)} gait={format_optional(gait)} "
          f"sub_gait={format_optional(sub_gait)} last_state={format_optional(last_state)} "
          f"last_gait={format_optional(last_gait)}")
    print(f"  max_vel={format_tuple(robot_state.getMaxVelocity())}")

    if bpw:
        print("  leg_odom=<unsupported>")
    else:
        leg_odom = robot_state.getLegOdom()
        if leg_odom is None:
            print("  leg_odom=<unavailable>")
        else:
            print(f"  leg_odom.position={format_tuple(leg_odom['position'])} "
                  f"velocity_body={format_tuple(leg_odom['velocity_body'])} "
                  f"orientation={format_tuple(leg_odom['orientation'])} "
                  f"angular_velocity={format_tuple(leg_odom['angular_velocity'])}")

    joint_timestamp = robot_state.getJointStateTimestamp()
    imu_timestamp = robot_state.getImuTimestamp()
    odometry_timestamp = "<unsupported>" if bpw else format_optional(robot_state.getOdometryTimestamp())
    motion_timestamp = robot_state.getMotionStateTimestamp()
    battery_timestamp = robot_state.getBatteryTimestamp()
    fault_timestamp = robot_state.getFaultTimestamp()
    print(f"  timestamps_ms joint={format_optional(joint_timestamp)} "
          f"imu={format_optional(imu_timestamp)} odometry={odometry_timestamp} "
          f"motion={format_optional(motion_timestamp)} battery={format_optional(battery_timestamp)} "
          f"fault={format_optional(fault_timestamp)}")

    motor_temperatures = robot_state.getMotorTemperature()
    driver_temperatures = robot_state.getDriverTemperature()
    print_joint_list("motor_temperature", motor_temperatures)
    print_joint_list("driver_temperature", driver_temperatures)
    print_joint_temperatures(robot_state)
    print(f"  {format_power_state(robot_state)}")
    print_fault_details(robot_state)


def main():
    options = parse_options(False)
    print_version()
    robot_state = mirrorme_robot_sdk.RequestRobotState()
    robot_state.setRobotIp(options.robot_ip)
    robot_state.setRobotStateUploadPort(options.robot_state_port)
    robot_state.setTcpLocalPort(options.tcp_local_port)
    robot_state.setRobotStateUploadRate(options.state_rate_hz)
    print(
        f"robot_ip={options.robot_ip} "
        f"state_port={options.robot_state_port} "
        f"tcp_local_port={options.tcp_local_port} "
        f"state_rate={options.state_rate_hz}"
    )

    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    previous_handlers = {
        signum: signal.signal(signum, request_stop)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        print("connection: connecting", flush=True)
        if not robot_state.connect():
            raise RuntimeError("failed to connect request robot state")
        was_connected = False
        ever_connected = False
        last_model = None
        while not stop_requested:
            connected = bool(robot_state.isConnected())
            if connected != was_connected:
                if connected:
                    event = "reconnected" if ever_connected else "connected"
                    print(f"connection: {event}", flush=True)
                    ever_connected = True
                    print_robot_version(robot_state)
                    print_identity_availability(robot_state, True)
                else:
                    print("connection: disconnected; retrying", flush=True)
                was_connected = connected

            print(f"robot state: {datetime.now().isoformat(sep=' ', timespec='milliseconds')}")
            print(f"  connected={str(connected).lower()}")
            if connected:
                model = robot_state.getRobotModel()
                if model is not None:
                    last_model = model
                print_telemetry(robot_state, last_model == mirrorme_robot_sdk.RobotModel.BPW)
            else:
                print("  telemetry=<unavailable: disconnected>")
            print(flush=True)
            time.sleep(0.2)
    finally:
        robot_state.disconnect()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
        print("connection: stopped", flush=True)


if __name__ == "__main__":
    main()

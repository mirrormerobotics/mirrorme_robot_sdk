import signal
import sys
import time
from enum import Enum
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mirrorme_robot_sdk
from example_options import parse_options
from example_feedback import format_power_state, format_control_mode, print_identity_availability


class DemoPhase(Enum):
    CONNECTING = 0
    PREPARE = 1
    STAND_UP = 2
    WALK = 3
    YAW_VELOCITY = 4
    ZERO_VELOCITY = 5
    SIT_DOWN = 6


def print_version():
    print(f"mirrorme_robot_sdk version={getattr(mirrorme_robot_sdk, '__version__', 'unknown')}", flush=True)


def print_robot_version(motion):
    version = motion.getRobotVersion()
    if version is None:
        print("robot version (cached): <unavailable>", flush=True)
        return
    major, minor, patch, commit, build_date, build_time = version
    print(f"robot version (cached): {major}.{minor}.{patch} commit=0x{commit:x} "
          f"build={build_date}T{build_time:06d}", flush=True)


def print_status(motion, connected, state, gait, timestamp):
    if not connected:
        print("motion_state=<unavailable> gait=<unavailable> control_mode=<unavailable> "
              "motion_timestamp_ms=<unavailable>", flush=True)
        return
    max_velocity = motion.getMaxVelocity()
    max_text = "<unavailable>" if max_velocity is None else "(" + ", ".join(
        f"{value:.6g}" for value in max_velocity) + ")"
    print(f"motion_state={state if state is not None else '<unavailable>'} "
          f"gait={gait if gait is not None else '<unavailable>'} "
          f"control_mode={format_control_mode(motion)} {format_power_state(motion)} "
          f"max_vel={max_text} "
          f"motion_timestamp_ms={timestamp if timestamp is not None else '<unavailable>'}", flush=True)


def main():
    options = parse_options(False)
    print_version()
    motion = mirrorme_robot_sdk.MotionLevelControl()
    motion.setRobotIp(options.robot_ip)
    motion.setRobotStateUploadPort(options.robot_state_port)
    motion.setTcpLocalPort(options.tcp_local_port)
    motion.setRobotStateUploadRate(options.state_rate_hz)
    motion.setMotionCommandRate(50)
    print(f"robot_ip={options.robot_ip} state_port={options.robot_state_port} "
          f"tcp_local_port={options.tcp_local_port} state_rate={options.state_rate_hz}", flush=True)

    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    previous_handlers = {signum: signal.signal(signum, request_stop)
                         for signum in (signal.SIGINT, signal.SIGTERM)}
    phase = DemoPhase.CONNECTING
    phase_started = time.monotonic()
    phase_baseline = None
    first_command_sent = False
    prepare_command_time = None
    yaw_started = None
    last_motion_timestamp = None
    last_motion_progress = phase_started
    next_status = phase_started
    seen_connected = False
    model = None
    result = 0
    phase_logs = {
        DemoPhase.PREPARE: ("prepare", None),
        DemoPhase.STAND_UP: ("stand-up", "send stand-up command"),
        DemoPhase.WALK: ("walk", "select walk gait; waiting for fresh feedback"),
        DemoPhase.YAW_VELOCITY: ("yaw", "send yaw velocity command: 1.0 rad/s for 3 seconds"),
        DemoPhase.ZERO_VELOCITY: ("zero-velocity", "send zero velocity command for 2 seconds"),
        DemoPhase.SIT_DOWN: ("sit-down", "send sit-down command; waiting for fresh feedback"),
    }

    def enter_phase(next_phase, now, timestamp):
        nonlocal phase, phase_started, phase_baseline, first_command_sent
        phase = next_phase
        phase_started = now
        phase_baseline = timestamp
        first_command_sent = False
        name, message = phase_logs[phase]
        print(f"phase: {name}", flush=True)
        if message is not None:
            print(message, flush=True)

    try:
        print("connection: connecting", flush=True)
        if not motion.connect():
            if not stop_requested:
                raise RuntimeError("failed to connect motion level control")
            return 0
        while not stop_requested:
            now = time.monotonic()
            connected = bool(motion.isConnected())
            if connected:
                seen_connected = True
            elif seen_connected or phase != DemoPhase.CONNECTING:
                raise RuntimeError("connection lost")
            state = motion.getCurrentMotionState() if connected else None
            gait = motion.getCurrentGait() if connected else None
            timestamp = motion.getMotionStateTimestamp() if connected else None
            has_feedback = state is not None and gait is not None and timestamp is not None
            motion_progressed = timestamp is not None and timestamp != last_motion_timestamp

            if phase != DemoPhase.CONNECTING:
                if motion_progressed:
                    last_motion_timestamp = timestamp
                    last_motion_progress = now
                if now - last_motion_progress >= 1.0:
                    raise RuntimeError("motion feedback timeout")
            if now >= next_status:
                print_status(motion, connected, state, gait, timestamp)
                next_status = now + 0.2

            if phase == DemoPhase.CONNECTING:
                if now - phase_started >= 10.0:
                    raise RuntimeError("connection timeout")
                model = motion.getRobotModel() if connected else None
                if has_feedback and model in (mirrorme_robot_sdk.RobotModel.BPX,
                                               mirrorme_robot_sdk.RobotModel.BPXPro,
                                               mirrorme_robot_sdk.RobotModel.BPW):
                    print("connection: connected", flush=True)
                    print_robot_version(motion)
                    print_identity_availability(motion, True)
                    motion.setVelocityControlFlag(True)
                    last_motion_timestamp = timestamp
                    last_motion_progress = now
                    enter_phase(DemoPhase.PREPARE, now, timestamp)

            elif phase == DemoPhase.PREPARE:
                if prepare_command_time is None and now - phase_started >= 0.02:
                    # Let the SDK send its initial zero baseline before this one-shot flag.
                    if model == mirrorme_robot_sdk.RobotModel.BPW:
                        print("BPW: skip zero-position command", flush=True)
                    else:
                        motion.setZeroPositionsFlag()
                        print("send zero-position command", flush=True)
                    print("wait 1 second before sending commands", flush=True)
                    prepare_command_time = time.monotonic()
                elif prepare_command_time is not None and now - prepare_command_time >= 1.0:
                    enter_phase(DemoPhase.STAND_UP, now, timestamp)

            elif phase == DemoPhase.STAND_UP:
                if now - phase_started >= 10.0:
                    raise RuntimeError("stand-up timeout")
                if not motion.setStandUp():
                    raise RuntimeError("failed to send stand-up command")
                if not first_command_sent:
                    phase_baseline = motion.getMotionStateTimestamp()
                    first_command_sent = True
                elif motion_progressed and has_feedback and timestamp != phase_baseline and state == mirrorme_robot_sdk.MotionState.Motion:
                    enter_phase(DemoPhase.WALK, now, timestamp)

            elif phase == DemoPhase.WALK:
                if now - phase_started >= 10.0:
                    raise RuntimeError("walk gait timeout")
                motion.setWalk()
                if not motion.setVelocity(0.0, 0.0, 0.0):
                    raise RuntimeError("failed to send walk zero velocity command")
                if not first_command_sent:
                    phase_baseline = motion.getMotionStateTimestamp()
                    first_command_sent = True
                elif (motion_progressed and has_feedback and timestamp != phase_baseline and
                      state == mirrorme_robot_sdk.MotionState.Motion and gait == mirrorme_robot_sdk.MotionGait.Walk):
                    enter_phase(DemoPhase.YAW_VELOCITY, now, timestamp)

            elif phase == DemoPhase.YAW_VELOCITY:
                if yaw_started is not None and now - yaw_started >= 3.0:
                    if not motion.setVelocity(0.0, 0.0, 0.0):
                        raise RuntimeError("failed to send zero velocity command")
                    enter_phase(DemoPhase.ZERO_VELOCITY, time.monotonic(), timestamp)
                else:
                    if not motion.setVelocity(0.0, 0.0, 1.0):
                        raise RuntimeError("failed to send yaw velocity command")
                    if yaw_started is None:
                        yaw_started = time.monotonic()

            elif phase == DemoPhase.ZERO_VELOCITY:
                if not motion.setVelocity(0.0, 0.0, 0.0):
                    raise RuntimeError("failed to send zero velocity command")
                if now - phase_started >= 2.0:
                    enter_phase(DemoPhase.SIT_DOWN, now, timestamp)

            elif phase == DemoPhase.SIT_DOWN:
                if now - phase_started >= 10.0:
                    raise RuntimeError("sit-down timeout")
                if not motion.setSitDown():
                    raise RuntimeError("failed to send sit-down command")
                if not first_command_sent:
                    phase_baseline = motion.getMotionStateTimestamp()
                    first_command_sent = True
                elif motion_progressed and has_feedback and timestamp != phase_baseline and state == mirrorme_robot_sdk.MotionState.LyingDown:
                    print("motion demo: completed", flush=True)
                    break
            time.sleep(0.02)
    except RuntimeError as error:
        if not stop_requested:
            print(str(error), file=sys.stderr, flush=True)
            result = 1
    finally:
        try:
            if phase in (DemoPhase.WALK, DemoPhase.YAW_VELOCITY, DemoPhase.ZERO_VELOCITY) and motion.isConnected():
                motion.setVelocity(0.0, 0.0, 0.0)
        except RuntimeError:
            pass
        finally:
            motion.disconnect()
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
            print("connection: stopped", flush=True)
    return result


if __name__ == "__main__":
    sys.exit(main())

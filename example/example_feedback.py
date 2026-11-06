"""Formatting shared by the robot state and control examples."""
import mirrorme_robot_sdk


def is_bpw(robot):
    return robot.getRobotModel() == mirrorme_robot_sdk.RobotModel.BPW


def print_identity_availability(robot, show_identity=False):
    sn = robot.getRobotSerialNumber()
    model = robot.getRobotModel()
    if show_identity:
        sn_text = "<unavailable>" if sn is None else sn or "<empty>"
        model_text = "<unavailable>"
        if model is not None:
            try:
                model_text = f"{mirrorme_robot_sdk.RobotModel(model).name} ({model})"
            except ValueError:
                model_text = str(model)
        print(f"robot identity: serial_number={sn_text} model={model_text}", flush=True)
    elif sn is None or model is None:
        print("robot identity: <unavailable>", flush=True)
    if model == mirrorme_robot_sdk.RobotModel.BPW:
        print("BPW: basic state and motion (walk=flat, rough) supported; "
              "fixed 12-joint state/temperature, leg odometry and JointLevelControl unsupported; dynamic joint state/temperature feedback is negotiated automatically.", flush=True)


def format_control_mode(robot):
    mode = robot.getControlMode()
    if mode is None:
        return "<unavailable>"
    return f"{mirrorme_robot_sdk.ControlMode(mode).name} ({mode})"


def format_joint_count(robot):
    snapshot = robot.getJointStates()
    if snapshot is None:
        snapshot = robot.getJointTemperatures()
    return str(len(snapshot['joints'])) if snapshot is not None else "unknown"


def format_number(value):
    return "<unavailable>" if value is None else f"{value:.6g}"


def format_optional(value):
    return "<unavailable>" if value is None else str(value)


def format_tuple(values):
    if values is None:
        return "<unavailable>"
    return "(" + ", ".join(format_number(value) for value in values) + ")"


def print_joint_list(name, values):
    if values is None:
        print(f"  {name}=<unavailable>")
        return
    print(f"  {name} count={len(values)} values=[" +
          ", ".join(format_number(value) for value in values) + "]")


def print_joint_states(robot):
    snapshot = robot.getJointStates()
    if snapshot is None:
        print("  joint_states=<unavailable>")
        return
    print(f"  joint_states count={len(snapshot['joints'])} "
          f"timestamp_ms={snapshot['timestamp_ms']} age_ms={snapshot['age_ms']} "
          f"sequence={snapshot['sequence']} topology_revision={snapshot['topology_revision']}")
    for joint in snapshot['joints']:
        values = {name: format_number(joint[name])
                  for name in ('position', 'velocity', 'torque')}
        print(f"    joint_id={joint['joint_id']} position={values['position']} "
              f"velocity={values['velocity']} torque={values['torque']}")


def print_joint_temperatures(robot):
    snapshot = robot.getJointTemperatures()
    if snapshot is None:
        print("  joint_temperatures=<unavailable>")
        return
    print(f"  joint_temperatures count={len(snapshot['joints'])} "
          f"timestamp_ms={snapshot['timestamp_ms']} age_ms={snapshot['age_ms']} "
          f"sequence={snapshot['sequence']} topology_revision={snapshot['topology_revision']}")
    for joint in snapshot['joints']:
        motor, driver = joint['motor_celsius'], joint['driver_celsius']
        print(f"    joint_id={joint['joint_id']} "
              f"motor_celsius={format_number(motor)} "
              f"driver_celsius={format_number(driver)}")


def print_fault_details(robot):
    faults = robot.getFaultState()
    if faults is None:
        print("  fault_details=<unavailable>")
        return
    print(f"  fault_details count={faults['entry_count']} "
          f"truncated={str(bool(faults['truncated'])).lower()}")
    for index, fault in enumerate(faults['entries']):
        print(f"    fault index={index} code=0x{fault['code']:x} "
              f"occurred_t_ms={fault['occurred_t_ms']}")


def format_power_state(robot):
    level = robot.getBatteryLevel()
    current = robot.getBatteryCurrent()
    in1 = robot.getChargerIn1()
    in2 = robot.getChargerIn2()
    estop = robot.getHardEstop()
    faults = robot.getFaultState()
    level_text = f"{level}%" if level is not None else "<unavailable>"
    current_text = f"{current:.3f}" if current is not None else "<unavailable>"
    if faults is None:
        fault_text = "<unavailable>"
    else:
        fault_text = f"{faults['entry_count']}{'+' if faults['truncated'] else ''}"
        if faults["entries"]:
            fault_text += f" top=0x{faults['entries'][0]['code']:x}"
    return (f"battery_level={level_text} battery_current={current_text} "
            f"charger_in1={in1 if in1 is not None else '<unavailable>'} "
            f"charger_in2={in2 if in2 is not None else '<unavailable>'} "
            f"hard_estop={estop if estop is not None else '<unavailable>'} "
            f"faults={fault_text}")

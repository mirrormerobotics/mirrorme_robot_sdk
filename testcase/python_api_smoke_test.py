"""Verify the installed wheel exposes the public Python robot APIs."""
import mirrorme_robot_sdk

assert [(model.name, model.value) for model in mirrorme_robot_sdk.RobotModel] == [
    ("Unknown", 0), ("BPX", 1), ("BPXPro", 2), ("BPW", 3)]
assert [(mode.name, mode.value) for mode in mirrorme_robot_sdk.ControlMode] == [
    ("Unknown", 0), ("RemoteControl", 1), ("Navigator", 2)]
assert mirrorme_robot_sdk.MotionGait.Jump == 12
assert mirrorme_robot_sdk.MotionState.ChargingSitDown == 24
for kind in (mirrorme_robot_sdk.RequestRobotState, mirrorme_robot_sdk.MotionLevelControl, mirrorme_robot_sdk.JointLevelControl):
    robot = kind()
    assert robot.getRobotSerialNumber() is None
    assert robot.getRobotModel() is None
    assert robot.getControlMode() is None
    assert robot.getChargerIn1() is None
    assert robot.getChargerIn2() is None
motion = mirrorme_robot_sdk.MotionLevelControl()
assert not hasattr(motion, "setUpright")
assert mirrorme_robot_sdk.MotionGait.PoseTracking == 7
assert mirrorme_robot_sdk.MotionGait.Tumbler == 11
assert motion.setTumbler() is False
assert motion.setPoseTracking() is False
assert motion.setPose(0.0, 0.0, 0.0, 0.0) is False
assert motion.setChargingSitDown() is False
for name in ("setUpJump", "setFrontJump", "setBackJump", "setLeftJump", "setRightJump"):
    assert callable(getattr(motion, name))
print("Python SDK API smoke test passed")

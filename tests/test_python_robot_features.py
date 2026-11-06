"""Exercise Python bindings against a loopback peer; no robot hardware is used."""
import multiprocessing
import os
import select
import socket
import struct
import tempfile
import time
import unittest

import mirrorme_robot_sdk
from capability_wire import DISCOVERY_TAG, catalog, response as capability_response, selection_features


def peer(ready):
    clients = {}
    requests = []
    identity = (0, "original-SN")
    control_mode = 0
    capabilities = None
    capability_requests = []
    next_token = 0x12340000
    pause_accept = False
    server = socket.socket()
    try:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('127.0.0.1', 10860))
        server.listen()
        ready.send(None)
        while True:
            readable, _, _ = select.select([ready, *([] if pause_accept else [server]), *clients], [], [], 0.02)
            for sock in readable:
                if sock is ready:
                    message = ready.recv()
                    if message == 'drop_clients':
                        for client in list(clients):
                            client.close()
                        clients.clear()
                        ready.send(None)
                        continue
                    if isinstance(message, dict) and 'pause_accept' in message:
                        pause_accept = bool(message['pause_accept'])
                        ready.send(None)
                        continue
                    if message == 'requests':
                        ready.send(list(requests))
                        continue
                    if message == 'clear_requests':
                        requests.clear()
                        ready.send(None)
                        continue
                    if message == 'capability_sessions':
                        ready.send([state[3] for state in clients.values() if state[3] is not None])
                        continue
                    if message == 'capability_requests':
                        ready.send(list(capability_requests))
                        continue
                    if isinstance(message, dict) and 'capabilities' in message:
                        capabilities = message['capabilities']
                        capability_requests.clear()
                        ready.send(None)
                        continue
                    identity, control_mode = message
                    capabilities = None
                    capability_requests.clear()
                    ready.send(None)
                    continue
                if sock is server:
                    if pause_accept:
                        continue
                    client, _ = server.accept()
                    next_token += 1
                    clients[client] = [b'', None, next_token, None]
                    continue
                # A drop command can close sockets already returned by select.
                if sock not in clients:
                    continue
                try:
                    data = sock.recv(1024)
                except ConnectionResetError:
                    data = b''
                if not data:
                    sock.close()
                    del clients[sock]
                    continue
                clients[sock][0] += data
                while len(clients[sock][0]) >= 24:
                    req = clients[sock][0][:24]
                    session = struct.unpack_from('<H', req)[0]
                    mode = req[10]
                    body_size = struct.unpack_from('<I', req, 20)[0] if mode == 7 else 0
                    if len(clients[sock][0]) < 24 + body_size:
                        break
                    body = clients[sock][0][24:24 + body_size]
                    clients[sock][0] = clients[sock][0][24 + body_size:]
                    requests.append(req)
                    if mode == 7 and capabilities is not None:
                        operation = struct.unpack_from('<I', req, 16)[0]
                        request_id = struct.unpack_from('<I', req, 12)[0]
                        features = selection_features(body) if operation == 2 else []
                        capability_requests.append((operation, features))
                        token = clients[sock][2]
                        sock.sendall(capability_response(session, operation, request_id, token,
                                                        catalog(capabilities, features)))
                        if operation == 2:
                            state = clients[sock][3]
                            if state is not None:
                                state.update(token=token, enabled=features,
                                             topology_revision=capabilities['topology_revision'])
                    elif mode == 4:
                        response = bytearray(64)
                        struct.pack_into('<BBH', response, 0, 4, 1, session)
                        sock.sendall(response)
                    elif mode == 6:
                        wire_model, sn = identity
                        sock.sendall(struct.pack('<BBHBBBB32s56s',
                            6, 1, session, 1, wire_model, len(sn), 0, sn.encode(), b''))
                    elif mode == 5:
                        response = bytearray(32)
                        struct.pack_into('<BBH', response, 0, 5, 0, session)
                        response[28] = 2
                        sock.sendall(response)
                    else:
                        response = bytearray(32)
                        struct.pack_into('<HBxHBB', response, 0, session, 1, 50, mode, 3)
                        clients[sock][1] = response
                        clients[sock][3] = {'port': struct.unpack_from('<H', req, 2)[0],
                                             'token': clients[sock][2], 'enabled': [], 'mode': mode}
            for sock, state in list(clients.items()):
                response = state[1]
                if response is not None:
                    try:
                        struct.pack_into('<III', response, 20,
                                         0x434D0001 if control_mode is not None else 0,
                                         control_mode if control_mode is not None else 0,
                                         DISCOVERY_TAG if capabilities is not None else 0)
                        sock.sendall(response)
                    except OSError:
                        sock.close()
                        del clients[sock]
    except Exception as error:
        ready.send(repr(error))
        raise
    finally:
        server.close()


class RobotFeaturesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        context = multiprocessing.get_context('spawn')
        read, write = context.Pipe()
        cls.peer = context.Process(target=peer, args=(write,), daemon=True)
        cls.peer.start()
        if not read.poll(5):
            cls.peer.terminate()
            raise RuntimeError('loopback peer did not start')
        error = read.recv()
        cls.channel = read
        write.close()
        if error:
            cls.peer.join(1)
            raise RuntimeError(error)

    @classmethod
    def tearDownClass(cls):
        cls.peer.terminate()
        cls.peer.join(5)
        cls.channel.close()

    def wait(self, predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        self.assertTrue(predicate())

    def connect(self, kind):
        sdk = kind()
        self.addCleanup(sdk.disconnect)
        sdk.setRobotIp('127.0.0.1')
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        sdk.setRobotStateUploadPort(port)
        if isinstance(sdk, mirrorme_robot_sdk.JointLevelControl):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
                reservation.bind(('127.0.0.1', 0))
                sdk.setJointStateUploadPort(reservation.getsockname()[1])
        sdk.setRobotStateUploadRate(50)
        self.assertTrue(sdk.connect())
        self.assertIsNotNone(sdk.getRobotSerialNumber())
        self.assertIsNotNone(sdk.getRobotModel())
        self.wait(sdk.isConnected)
        return sdk, port

    def configure_peer(self, wire_model=0, sn='original-SN', control_mode=0):
        self.channel.send(((wire_model, sn), control_mode))
        self.assertTrue(self.channel.poll(3), 'peer configuration timed out')
        self.assertIsNone(self.channel.recv())

    def peer_requests(self, clear=False):
        self.channel.send('clear_requests' if clear else 'requests')
        self.assertTrue(self.channel.poll(3), 'request audit timed out')
        return self.channel.recv()

    def test_bpw_basic_state_and_motion(self):
        self.configure_peer(3, 'BPW-SN')
        self.addCleanup(self.configure_peer)
        self.assertFalse(mirrorme_robot_sdk.MotionLevelControl().setRough())
        for kind in (mirrorme_robot_sdk.RequestRobotState, mirrorme_robot_sdk.MotionLevelControl):
            sdk, port = self.connect(kind)
            self.assertEqual(sdk.getRobotModel(), mirrorme_robot_sdk.RobotModel.BPW)
            self.assertEqual(sdk.getRobotSerialNumber(), 'BPW-SN')
            self.assertIsNotNone(sdk.getRobotVersion())
            self.wait(lambda: sdk.getControlMode() == mirrorme_robot_sdk.ControlMode.RemoteControl)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                def upload(head, payload, timestamp=10):
                    sender.sendto(struct.pack('<HHI', head, len(payload), timestamp) + payload,
                                  ('127.0.0.1', port))
                upload(0x0200, struct.pack('<13f', .1, .2, .3, 0, 0, 0, 1,
                                         0, 0, 9.8, .4, .5, .6))
                upload(0x0001, struct.pack('<BB2xf24b', 87, 7, -2.5, *([30] * 24)))
                upload(0x00f0, bytes(520))
                # A legacy peer must not make fixed 12-axis or odometry APIs appear supported.
                upload(0x1000, struct.pack('<36f', *range(36)))
                upload(0x0050, struct.pack('<13f', *range(13)))
                self.wait(lambda: sdk.getImuTimestamp() == 10 and sdk.getBatteryTimestamp() == 10)
                self.wait(lambda: sdk.getFaultTimestamp() == 10)
                self.assertAlmostEqual(sdk.getImuRpy()[0], .1)
                self.assertEqual(sdk.getImuQuat(), [0, 0, 0, 1])
                self.assertAlmostEqual(sdk.getImuAcc()[2], 9.8, places=5)
                self.assertAlmostEqual(sdk.getImuOmega()[1], .5)
                self.assertEqual((sdk.getBatteryLevel(), sdk.getBatteryCurrent(), sdk.getChargerIn1(),
                                  sdk.getChargerIn2(), sdk.getHardEstop()), (87, -2.5, 1, 1, 1))
                self.assertEqual(sdk.getFaultState()['entry_count'], 0)
                for timestamp, gait in [(11, 16), (12, 0)]:
                    upload(0x0010, struct.pack('<5B3x3f', 6, gait, 1, 0, 0, 1.5, 1, 1), timestamp)
                    self.wait(lambda: sdk.getMotionStateTimestamp() == timestamp)
                    self.assertEqual((sdk.getCurrentMotionState(), sdk.getLastMotionState(),
                                      sdk.getCurrentGait(), sdk.getLastGait(), sdk.getSubGait()),
                                     (6, 1, gait, 0, 0))
                    self.assertEqual(sdk.getMaxVelocity(), [1.5, 1, 1])
                previous_stderr = os.dup(2)
                try:
                    with tempfile.TemporaryFile() as captured:
                        os.dup2(captured.fileno(), 2)
                        for getter in ('getJointStates', 'getJointPosition', 'getJointVelocity', 'getJointTorque',
                                       'getMotorTemperature', 'getDriverTemperature',
                                       'getJointStateTimestamp', 'getLegOdom', 'getOdometryTimestamp'):
                            self.assertIsNone(getattr(sdk, getter)(), getter)
                        os.dup2(previous_stderr, 2)
                        captured.seek(0)
                        diagnostic = captured.read().decode('utf-8')
                        self.assertIn('unsupported', diagnostic)
                        self.assertTrue(diagnostic.isascii())
                finally:
                    os.dup2(previous_stderr, 2)
                    os.close(previous_stderr)
            sdk.disconnect()

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 9527))
            receiver.settimeout(.1)
            sdk, _ = self.connect(mirrorme_robot_sdk.MotionLevelControl)
            def receive_matching(predicate):
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    try:
                        packet = receiver.recv(1024)
                    except socket.timeout:
                        continue
                    self.assertEqual(len(packet), 56)
                    if predicate(packet):
                        return packet
                self.fail('expected BPW motion command did not arrive')
            # TCP acceptance precedes the sender's first safe 20 ms baseline.
            # Discard packets queued before acceptance, then observe a fresh
            # zero frame without issuing a business command to probe readiness.
            receiver.setblocking(False)
            try:
                while True:
                    receiver.recv(1024)
            except BlockingIOError:
                pass
            finally:
                receiver.settimeout(.1)
            baseline = receive_matching(lambda packet: packet[4] == 6 and packet[5] == 0 and
                                        struct.unpack_from('<6f', packet, 8) == (0,) * 6)
            self.assertEqual(baseline[34], 0)
            self.assertTrue(sdk.setStandUp())
            receive_matching(lambda packet: packet[4] == 1)
            self.assertTrue(sdk.setRough())
            rough = receive_matching(lambda packet: packet[5] == 16)
            self.assertEqual((rough[4], rough[34]), (6, 0))
            self.assertEqual(struct.unpack_from('<6f', rough, 8), (0,) * 6)
            sdk.setVelocityControlFlag(True)
            self.assertTrue(sdk.setVelocity(.2, -.1, .3))
            packet = receive_matching(lambda packet: packet[5] == 16 and
                                      abs(struct.unpack_from('<f', packet, 8)[0] - .2) < .001)
            version = struct.unpack_from('<I', packet, 40)[0]
            for name in ('setRunning', 'setTrotRun', 'setBipedal', 'setInvBipedal', 'setPronk',
                         'setPace', 'setBound', 'setLeftFlip', 'setRightFlip', 'setUpJump',
                         'setFrontJump', 'setBackJump', 'setLeftJump', 'setRightJump', 'setZeroPositionsFlag'):
                getattr(sdk, name)()
            for name in ('setChargingSitDown', 'setTumbler', 'setPoseTracking'):
                self.assertFalse(getattr(sdk, name)())
            self.assertFalse(sdk.setPose(0, 0, 0, 0))
            deadline = time.monotonic() + .15
            while time.monotonic() < deadline:
                packet = receiver.recv(1024)
                self.assertEqual((packet[4], packet[5], packet[33]), (6, 16, 0))
                self.assertEqual(struct.unpack_from('<I', packet, 40)[0], version)
            sdk.setWalk()
            receive_matching(lambda packet: packet[5] == 0)
            self.assertTrue(sdk.setSitDown())
            receive_matching(lambda packet: packet[4] == 3)
            self.assertTrue(sdk.setDamping())
            receive_matching(lambda packet: packet[4] == 2)
            sdk.disconnect()
            self.assertFalse(sdk.setRough())

        for wire_model in (0, 1, 2):
            self.configure_peer(wire_model)
            sdk, _ = self.connect(mirrorme_robot_sdk.MotionLevelControl)
            self.assertFalse(sdk.setRough())
            sdk.disconnect()

    def test_bpw_joint_level_rejected_without_joint_subscription_or_udp(self):
        self.configure_peer(3, 'BPW-SN')
        self.addCleanup(self.configure_peer)
        self.peer_requests(clear=True)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 7896))
            receiver.settimeout(.15)
            sdk = mirrorme_robot_sdk.JointLevelControl()
            self.addCleanup(sdk.disconnect)
            sdk.setRobotIp('127.0.0.1')
            self.assertFalse(sdk.connect())
            self.assertFalse(sdk.isConnected())
            self.assertFalse(sdk.setJointCommand(*([[0] * 12] * 5)))
            self.assertFalse(sdk.setZeroJointCommand())
            for name in ('setJointKp', 'setJointPosition', 'setJointKd', 'setJointVelocity',
                         'setJointTorqueFeedForward'):
                self.assertFalse(getattr(sdk, name)([0] * 12))
            for name in ('getJointPositionHighRate', 'getJointVelocityHighRate',
                         'getJointTorqueHighRate', 'getImuRpyHighRate', 'getImuQuatHighRate',
                         'getImuAccHighRate', 'getImuOmegaHighRate',
                         'getJointStateTimestampHighRate', 'getJointStateSeqHighRate'):
                self.assertIsNone(getattr(sdk, name)())
            sdk.disconnect()
            with self.assertRaises(socket.timeout):
                receiver.recv(1024)
        self.assertTrue(self.peer_requests())
        self.assertNotIn(3, [request[10] for request in self.peer_requests()])

    def test_identity_on_all_three_classes(self):
        for kind in (mirrorme_robot_sdk.RequestRobotState, mirrorme_robot_sdk.MotionLevelControl, mirrorme_robot_sdk.JointLevelControl):
            for wire, model, sn in [(0, mirrorme_robot_sdk.RobotModel.BPX, 'original-SN'),
                                    (1, mirrorme_robot_sdk.RobotModel.BPXPro, 'S' * 32),
                                    (2, mirrorme_robot_sdk.RobotModel.Unknown, ''),
                                    (3, mirrorme_robot_sdk.RobotModel.BPW, 'BPW-SN'),
                                    (255, mirrorme_robot_sdk.RobotModel.Unknown, 'future-SN')]:
                with self.subTest(kind=kind.__name__, model=model, sn=sn):
                    self.configure_peer(wire, sn)
                    if kind is mirrorme_robot_sdk.JointLevelControl and wire == 3:
                        sdk = kind()
                        sdk.setRobotIp('127.0.0.1')
                        self.assertFalse(sdk.connect())
                        self.assertFalse(sdk.isConnected())
                        sdk.disconnect()
                        continue
                    sdk, port = self.connect(kind)
                    self.assertEqual(sdk.getRobotSerialNumber(), sn)
                    self.assertEqual(mirrorme_robot_sdk.RobotModel(sdk.getRobotModel()), model)
                    sdk.disconnect()
                    self.assertIsNone(sdk.getRobotSerialNumber())
                    self.assertIsNone(sdk.getRobotModel())

    def test_control_mode_on_all_three_classes(self):
        for kind in (mirrorme_robot_sdk.RequestRobotState, mirrorme_robot_sdk.MotionLevelControl, mirrorme_robot_sdk.JointLevelControl):
            with self.subTest(kind=kind.__name__):
                self.configure_peer()
                sdk, _ = self.connect(kind)
                for wire, expected in [(0, mirrorme_robot_sdk.ControlMode.RemoteControl),
                                       (1, mirrorme_robot_sdk.ControlMode.Navigator),
                                       (255, mirrorme_robot_sdk.ControlMode.Unknown), (None, None)]:
                    self.configure_peer(control_mode=wire)
                    self.wait(lambda: sdk.getControlMode() == expected)
                    if expected is not None:
                        self.assertEqual(mirrorme_robot_sdk.ControlMode(sdk.getControlMode()), expected)
                sdk.disconnect()
                self.assertIsNone(sdk.getControlMode())

    def test_charger_feedback_on_all_three_classes(self):
        for kind in (mirrorme_robot_sdk.RequestRobotState, mirrorme_robot_sdk.MotionLevelControl, mirrorme_robot_sdk.JointLevelControl):
            with self.subTest(kind=kind.__name__):
                self.configure_peer()
                sdk, port = self.connect(kind)
                self.assertIsNone(sdk.getChargerIn1())
                self.assertIsNone(sdk.getChargerIn2())
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                    for flags in (0, 1, 2, 3, 0xfc):
                        payload = bytearray(32)
                        payload[1] = flags
                        sender.sendto(struct.pack('<HHI', 1, 32, flags + 1) + payload,
                                      ('127.0.0.1', port))
                        self.wait(lambda: (sdk.getChargerIn1(), sdk.getChargerIn2()) ==
                                  (flags & 1, (flags >> 1) & 1))
                sdk.disconnect()

    def test_charging_sit_down_command_and_feedback(self):
        motion = mirrorme_robot_sdk.MotionLevelControl()
        self.assertIs(motion.setChargingSitDown(), False)
        self.configure_peer()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 9527))
            receiver.settimeout(2)
            sdk, port = self.connect(mirrorme_robot_sdk.MotionLevelControl)
            self.wait(lambda: sdk.setVelocity(0.3, 0.1, 0.2))
            self.assertIs(sdk.setChargingSitDown(), True)
            deadline = time.monotonic() + 3
            while True:
                self.assertLess(time.monotonic(), deadline)
                packet = receiver.recv(1024)
                if len(packet) == 56 and packet[4] == 24:
                    break
            self.assertEqual(packet[5], 0)  # Walk gait
            self.assertEqual(struct.unpack_from('<6f', packet, 8), (0.0,) * 6)
            self.assertIsNone(sdk.getCurrentMotionState())
            self.assertIsNone(sdk.getLastMotionState())
            for timestamp, current, previous in [(1, 24, 6), (2, 2, 24)]:
                payload = bytearray(20)
                payload[0], payload[2] = current, previous
                receiver.sendto(struct.pack('<HHI', 0x10, 20, timestamp) + payload,
                                ('127.0.0.1', port))
                self.wait(lambda: sdk.getMotionStateTimestamp() == timestamp)
                self.assertEqual(sdk.getCurrentMotionState(), current)
                self.assertEqual(sdk.getLastMotionState(), previous)
                self.assertEqual(mirrorme_robot_sdk.MotionState(24).name, 'ChargingSitDown')
            sdk.disconnect()
            self.assertIs(sdk.setChargingSitDown(), False)

    def test_pose_tracking_and_four_axis_input(self):
        axes = (0.2, -0.3, 1.0, -1.0)
        expected_axes = (0.2, -0.3, -1.0, 1.0, 0.0, 0.0)
        disconnected = mirrorme_robot_sdk.MotionLevelControl()
        self.assertIs(disconnected.setPoseTracking(), False)
        self.assertIs(disconnected.setPose(*axes), False)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 9527))
            receiver.settimeout(2)
            sdk, _ = self.connect(mirrorme_robot_sdk.MotionLevelControl)
            sdk.setVelocityControlFlag(True)
            self.wait(lambda: sdk.setVelocity(0.3, 0.1, 0.2))
            self.assertIs(sdk.setPose(*axes), False)

            def receive_matching(predicate):
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    packet = receiver.recv(1024)
                    self.assertEqual(len(packet), 56)
                    if predicate(packet):
                        return packet
                self.fail('expected pose packet did not arrive')

            self.assertIs(sdk.setPoseTracking(), True)
            packet = receive_matching(lambda p: p[5] == 7)
            self.assertEqual((packet[4], packet[32], packet[34]), (6, 0, 0))
            self.assertEqual(struct.unpack_from('<6f', packet, 8), (0.0,) * 6)
            selected_version = struct.unpack_from('<I', packet, 40)[0]
            self.assertIs(sdk.setPose(*axes), True)
            packet = receive_matching(lambda p: p[5] == 7 and
                struct.unpack_from('<I', p, 40)[0] != selected_version)
            self.assertEqual(packet[32], 0)
            for actual, expected in zip(struct.unpack_from('<6f', packet, 8), expected_axes):
                self.assertAlmostEqual(actual, expected)
            version = struct.unpack_from('<I', packet, 40)[0]
            for index in range(4):
                for invalid in (1.1, -1.1, float('nan'), float('inf'), -float('inf')):
                    bad_axes = list(axes)
                    bad_axes[index] = invalid
                    self.assertIs(sdk.setPose(*bad_axes), False)
            packet = receive_matching(lambda p: p[5] == 7)
            self.assertEqual(struct.unpack_from('<I', packet, 40)[0], version)
            for invalid in ((), (0, 0, 0), (0,) * 5, (0,) * 6, ([0] * 6,),
                            (None, 0, 0, 0), (0, 'bad', 0, 0)):
                with self.assertRaises(TypeError):
                    sdk.setPose(*invalid)
            sdk.setWalk()
            packet = receive_matching(lambda p: p[5] == 0)
            self.assertEqual(packet[32], 1)
            self.assertEqual(struct.unpack_from('<6f', packet, 8), (0.0,) * 6)
            self.assertIs(sdk.setPose(*axes), False)
            sdk.disconnect()
            self.assertIs(sdk.setPoseTracking(), False)

    def test_tumbler_self_righting_request(self):
        disconnected = mirrorme_robot_sdk.MotionLevelControl()
        self.assertIs(disconnected.setTumbler(), False)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 9527))
            receiver.settimeout(2)
            sdk, _ = self.connect(mirrorme_robot_sdk.MotionLevelControl)
            self.wait(lambda: sdk.setVelocity(0, 0, 0))
            self.assertIs(sdk.setTumbler(), True)
            deadline = time.monotonic() + 3
            while True:
                self.assertLess(time.monotonic(), deadline)
                packet = receiver.recv(1024)
                self.assertEqual(len(packet), 56)
                if packet[5] == 11:
                    break
            self.assertEqual(packet[4], 6)  # Motion state
            self.assertEqual(struct.unpack_from('b', packet, 34)[0], 0)
            self.assertEqual(struct.unpack_from('<6f', packet, 8), (0.0,) * 6)
            version = struct.unpack_from('<I', packet, 40)[0]
            self.assertIs(sdk.setTumbler(), True)
            deadline = time.monotonic() + 3
            while True:
                self.assertLess(time.monotonic(), deadline)
                packet = receiver.recv(1024)
                if packet[5] == 11 and struct.unpack_from('<I', packet, 40)[0] != version:
                    break
            sdk.disconnect()
            self.assertIs(sdk.setTumbler(), False)

    def test_jump_commands_and_repeated_requests(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 9527))
            receiver.settimeout(2)
            sdk, _ = self.connect(mirrorme_robot_sdk.MotionLevelControl)
            self.wait(lambda: sdk.setVelocity(0, 0, 0))
            previous_version = None
            for name, sub_gait in [('setUpJump', 0), ('setFrontJump', 1), ('setBackJump', 2),
                                    ('setLeftJump', -1), ('setRightJump', -2)]:
                for _ in range(2):
                    self.assertIsNone(getattr(sdk, name)())
                    deadline = time.monotonic() + 3
                    while True:
                        self.assertLess(time.monotonic(), deadline)
                        packet = receiver.recv(1024)
                        self.assertEqual(len(packet), 56)
                        version = struct.unpack_from('<I', packet, 40)[0]
                        if (packet[5] == 12 and struct.unpack_from('b', packet, 34)[0] == sub_gait
                                and version != previous_version):
                            break
                    previous_version = version
                    self.assertEqual(struct.unpack_from('<6f', packet, 8), (0.0,) * 6)
            sdk.disconnect()

    def test_enum_values_and_argument_validation(self):
        self.assertEqual([int(x) for x in mirrorme_robot_sdk.RobotModel], [0, 1, 2, 3])
        self.assertEqual([int(x) for x in mirrorme_robot_sdk.ControlMode], [0, 1, 2])
        self.assertEqual(mirrorme_robot_sdk.MotionGait.Tumbler, 11)
        self.assertEqual(mirrorme_robot_sdk.MotionGait.Jump, 12)
        self.assertEqual(mirrorme_robot_sdk.MotionGait.TrotRun, 15)
        self.assertEqual(mirrorme_robot_sdk.MotionGait.Rough, 16)
        self.assertEqual(mirrorme_robot_sdk.MotionState.ChargingSitDown, 24)
        sdk = mirrorme_robot_sdk.MotionLevelControl()
        self.assertFalse(hasattr(sdk, "setUpright"))
        for name in ('getRobotSerialNumber', 'getRobotModel', 'getJointStates', 'getJointTemperatures', 'getControlMode', 'getChargerIn1', 'getChargerIn2', 'setUpJump', 'setFrontJump',
                     'setBackJump', 'setLeftJump', 'setRightJump', 'setChargingSitDown', 'setTumbler', 'setRough', 'setTrotRun'):
            with self.assertRaises(TypeError):
                getattr(sdk, name)(1)


if __name__ == '__main__':
    unittest.main()

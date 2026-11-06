"""Loopback integration checks; never connects to robot hardware.

Set MIRRORME_ROBOT_SDK_EXAMPLE_BUILD to a CMake build directory to also check C++ examples.
"""
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest

import test_python_robot_features as robot_features
from capability_wire import temperatures, joint_profile, joint_states

ROOT = Path(__file__).resolve().parents[1]


class ServerClientExamplesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        robot_features.RobotFeaturesTest.setUpClass()

    @classmethod
    def tearDownClass(cls):
        robot_features.RobotFeaturesTest.tearDownClass()

    def test_request_state_example_temperature_support(self):
        build = os.environ.get('MIRRORME_ROBOT_SDK_EXAMPLE_BUILD')
        if not build:
            self.skipTest('set MIRRORME_ROBOT_SDK_EXAMPLE_BUILD to exercise the C++ CLI')
        executable = Path(build) / 'request_robot_state_example'
        self.assertTrue(executable.is_file(), str(executable))
        channel = robot_features.RobotFeaturesTest.channel
        # The fixed 12-axis wire values decode to 65 C / 67 C on BPX. BPW
        # must keep printing basic state without invoking these unsupported APIs.
        payload = struct.pack('<BBHf12b12b', 64, 1, 0, 1.25, *([5] * 12), *([7] * 12))
        packet = struct.pack('<HHI', 1, len(payload), 100) + payload
        power = 'battery_level=64% battery_current=1.250'
        mode = 'control_mode=RemoteControl (1)'

        def has_consecutive_basic_frames(output):
            valid = [power in frame and mode in frame
                     for frame in output.split('robot state: ')[1:]]
            return any(first and second for first, second in zip(valid, valid[1:]))

        try:
            for wire_model, sn in ((3, 'BPW-SN'), (0, 'original-SN')):
                with self.subTest(model=wire_model):
                    channel.send(((wire_model, sn), 0))
                    self.assertTrue(channel.poll(3), 'peer configuration timed out')
                    self.assertIsNone(channel.recv())
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
                        reservation.bind(('127.0.0.1', 0))
                        state_port = reservation.getsockname()[1]
                    with tempfile.TemporaryDirectory() as directory, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
                        stdout_path = Path(directory) / 'stdout.log'
                        stderr_path = Path(directory) / 'stderr.log'
                        with stdout_path.open('w') as stdout_log, stderr_path.open('w') as stderr_log:
                            process = subprocess.Popen(
                                [str(executable), '--robot-ip', '127.0.0.1', '--state-port', str(state_port)],
                                stdout=stdout_log, stderr=stderr_log)
                            try:
                                deadline = time.monotonic() + 5
                                while time.monotonic() < deadline and process.poll() is None:
                                    udp.sendto(packet, ('127.0.0.1', state_port))
                                    output = stdout_path.read_text()
                                    if (has_consecutive_basic_frames(output) and
                                            (wire_model == 3 or ('motor_temperature count=12 values=[65' in output and
                                                                 'driver_temperature count=12 values=[67' in output))):
                                        break
                                    time.sleep(.03)
                            finally:
                                if process.poll() is None:
                                    process.terminate()
                                try:
                                    process.communicate(timeout=5)
                                except subprocess.TimeoutExpired:
                                    process.kill()
                                    process.communicate(timeout=5)
                        output, errors = stdout_path.read_text(), stderr_path.read_text()
                        self.assertTrue(has_consecutive_basic_frames(output), output + errors)
                        if wire_model == 3:
                            self.assertNotIn('getMotorTemperature is unsupported on BPW in this SDK', errors)
                            self.assertNotIn('getDriverTemperature is unsupported on BPW in this SDK', errors)
                            self.assertNotIn('motor_temperature count=', output)
                        else:
                            self.assertIn('motor_temperature count=12 values=[65', output)
                            self.assertIn('driver_temperature count=12 values=[67', output)
        finally:
            channel.send(((0, 'original-SN'), 0))
            self.assertTrue(channel.poll(3), 'peer restore timed out')
            self.assertIsNone(channel.recv())

    def test_request_examples_show_dynamic_joint_states_and_partial_temperatures(self):
        channel = robot_features.RobotFeaturesTest.channel
        def ask(message):
            channel.send(message)
            self.assertTrue(channel.poll(3), 'peer configuration timed out')
            return channel.recv()
        self.assertIsNone(ask(((3, 'BPW-DYNAMIC'), 0)))
        config = joint_profile()
        self.assertIsNone(ask({'capabilities': config}))
        examples = [[sys.executable, '-u', str(ROOT / 'example/request_robot_state_example.py')]]
        if os.environ.get('MIRRORME_ROBOT_SDK_EXAMPLE_BUILD'):
            examples.append([str(Path(os.environ['MIRRORME_ROBOT_SDK_EXAMPLE_BUILD']) /
                                 'request_robot_state_example')])
        records = [(joint_id, float(index), None if index == 15 else 40.5 + index)
                   for index, (joint_id, _, _) in enumerate(config['joints'])]
        joint_records = [(joint_id, float(index), .5, None if index == 15 else -.5)
                         for index, (joint_id, _, _) in enumerate(config['joints'])]
        try:
            for example in examples:
                with self.subTest(example=example[0]):
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
                        reservation.bind(('127.0.0.1', 0))
                        state_port = reservation.getsockname()[1]
                    with tempfile.TemporaryDirectory() as directory, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
                        output_path = Path(directory) / 'stdout.log'
                        errors_path = Path(directory) / 'stderr.log'
                        with output_path.open('w') as stdout_log, errors_path.open('w') as stderr_log:
                            process = subprocess.Popen(example + ['--robot-ip', '127.0.0.1', '--state-port', str(state_port)],
                                                       stdout=stdout_log, stderr=stderr_log)
                            try:
                                deadline = time.monotonic() + 8
                                sequence = 1
                                while time.monotonic() < deadline and process.poll() is None:
                                    sessions = ask('capability_sessions')
                                    active = [item for item in sessions
                                              if item['port'] == state_port and item['enabled']]
                                    if active:
                                        state = max(active, key=lambda item: item['token'])
                                        for packet in temperatures(state['token'], 7, records, sequence=sequence):
                                            udp.sendto(packet, ('127.0.0.1', state_port))
                                        for packet in joint_states(state['token'], 7, joint_records, sequence=sequence):
                                            udp.sendto(packet, ('127.0.0.1', state_port))
                                        sequence += 1
                                    output = output_path.read_text()
                                    if all(text in output for text in
                                           ('joint_temperatures count=16', 'joint_states count=16', 'torque=<unavailable>',
                                            'joint_id=100', 'joint_id=145', 'driver_celsius=<unavailable>')):
                                        break
                                    time.sleep(.03)
                            finally:
                                if process.poll() is None:
                                    process.terminate()
                                try:
                                    process.communicate(timeout=5)
                                except subprocess.TimeoutExpired:
                                    process.kill()
                                    process.communicate(timeout=5)
                        output, errors = output_path.read_text(), errors_path.read_text()
                        for text in ('joint_temperatures count=16', 'joint_states count=16', 'torque=<unavailable>',
                                     'joint_id=100', 'joint_id=145', 'driver_celsius=<unavailable>'):
                            self.assertIn(text, output, output + errors)
                        self.assertNotIn('getMotorTemperature is unsupported', errors)
                        self.assertNotIn('getDriverTemperature is unsupported', errors)
        finally:
            self.assertIsNone(ask(((0, 'original-SN'), 0)))

    def test_status_joint_count_prefers_joint_state_then_temperature(self):
        import importlib.util
        module_path = ROOT / 'example/example_feedback.py'
        spec = importlib.util.spec_from_file_location('joint_feedback_test', module_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        class Feedback:
            states = None
            temperatures = None
            def getJointStates(self):
                return self.states
            def getJointTemperatures(self):
                return self.temperatures

        feedback = Feedback()
        self.assertEqual(module.format_joint_count(feedback), 'unknown')
        feedback.temperatures = {'joints': [{}] * 16}
        self.assertEqual(module.format_joint_count(feedback), '16')
        feedback.states = {'joints': [{}] * 64}
        self.assertEqual(module.format_joint_count(feedback), '64')
        feedback.states = None
        self.assertEqual(module.format_joint_count(feedback), '16')

    def test_interoperability(self):
        servers = [[sys.executable, str(ROOT / 'example/motion_level_control_server_example.py')]]
        clients = [[sys.executable, str(ROOT / 'example/motion_level_control_client_example.py')]]
        if os.environ.get('MIRRORME_ROBOT_SDK_EXAMPLE_BUILD'):
            build = Path(os.environ['MIRRORME_ROBOT_SDK_EXAMPLE_BUILD'])
            servers.append([str(build / 'motion_level_control_server_example')])
            clients.append([str(build / 'motion_level_control_client_example')])
        for server in servers:
            with self.subTest(server=server[0]):
                with socket.socket() as reservation:
                    reservation.bind(('127.0.0.1', 0))
                    port = reservation.getsockname()[1]
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
                    reservation.bind(('127.0.0.1', 0))
                    state_port = reservation.getsockname()[1]
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp, tempfile.TemporaryFile(mode='w+') as log:
                    udp.bind(('127.0.0.1', 9527))
                    udp.settimeout(2)
                    process = subprocess.Popen(server + ['--robot-ip', '127.0.0.1', '--state-port', str(state_port),
                                               '--listen-port', str(port)], stdout=log, stderr=log)
                    try:
                        deadline = time.monotonic() + 8
                        while True:
                            try:
                                with socket.create_connection(('127.0.0.1', port), timeout=.2) as conn:
                                    self.assertIn(b'OK', conn.recv(1024))
                                break
                            except OSError:
                                self.assertIsNone(process.poll())
                                self.assertLess(time.monotonic(), deadline)
                                time.sleep(.05)
                        def command(client, *words):
                            return subprocess.run(client + ['--server-port', str(port), *words],
                                                  capture_output=True, text=True, timeout=5)
                        for client in clients:
                            help_result = subprocess.run(client + ['--help'], capture_output=True,
                                                         text=True, timeout=5)
                            self.assertEqual(help_result.returncode, 0, help_result.stderr)
                            self.assertIn('charging-sit', help_result.stdout)
                            self.assertIn('tumbler', help_result.stdout)
                            self.assertIn('rough', help_result.stdout)
                            self.assertIn('trotrun', help_result.stdout)
                            remote_help = command(client, 'help')
                            self.assertEqual(remote_help.returncode, 0, remote_help.stdout + remote_help.stderr)
                            self.assertIn('charging-sit', remote_help.stdout)
                            self.assertIn('tumbler', remote_help.stdout)
                            for help_text in (help_result.stdout, remote_help.stdout):
                                self.assertRegex(help_text, r'\brun\b')
                                self.assertNotRegex(help_text, r'\brunning\b')
                            self.assertNotEqual(command(client, 'gait', 'rough').returncode, 0)
                            deadline = time.monotonic() + 5
                            while True:
                                result = command(client, 'status')
                                self.assertEqual(result.returncode, 0, result.stderr)
                                if 'connected=1' in result.stdout:
                                    break
                                self.assertLess(time.monotonic(), deadline)
                                time.sleep(.05)
                            for text in ('SN=original-SN', 'model=BPX', 'control_mode=RemoteControl (1)',
                                         'battery_level=', 'battery_current=', 'charger_in1=', 'charger_in2='):
                                self.assertIn(text, result.stdout)
                            for words in [('jump', 'invalid'), ('velocity', 'nan', '0', '0')]:
                                result = command(client, *words)
                                self.assertNotEqual(result.returncode, 0)
                                self.assertIn('ERR', result.stdout)
                            for direction, subgait in [('up',0),('front',1),('back',2),('left',-1),('right',-2)]:
                                result = command(client, 'jump', direction)
                                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                                deadline = time.monotonic() + 3
                                while True:
                                    packet = udp.recv(1024)
                                    if len(packet) == 56 and packet[5] == 12 and struct.unpack_from('b', packet, 34)[0] == subgait:
                                        break
                                    self.assertLess(time.monotonic(), deadline)
                            result = command(client, 'charging-sit')
                            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                            self.assertIn('OK charging sit-down command sent', result.stdout)
                            deadline = time.monotonic() + 3
                            while True:
                                self.assertLess(time.monotonic(), deadline)
                                packet = udp.recv(1024)
                                if len(packet) == 56 and packet[4] == 24:
                                    break
                            self.assertEqual(packet[5], 0)
                            self.assertEqual(struct.unpack_from('<6f', packet, 8), (0.0,) * 6)
                            result = command(client, 'tumbler')
                            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                            self.assertIn('OK tumbler command sent', result.stdout)
                            deadline = time.monotonic() + 3
                            while True:
                                self.assertLess(time.monotonic(), deadline)
                                packet = udp.recv(1024)
                                if len(packet) == 56 and packet[5] == 11:
                                    break
                            self.assertEqual(packet[4], 6)  # Motion state
                            self.assertEqual(struct.unpack_from('<6f', packet, 8), (0.0,) * 6)
                            for current, previous, expected in [
                                    (24, 6, ('motion_state=24(ChargingSitDown)', 'last_motion_state=6(Motion)')),
                                    (2, 24, ('motion_state=2(Passive)', 'last_motion_state=24(ChargingSitDown)'))]:
                                payload = bytearray(20)
                                payload[0], payload[2] = current, previous
                                deadline = time.monotonic() + 3
                                while True:
                                    udp.sendto(struct.pack('<HHI', 0x10, 20, current) + payload,
                                               ('127.0.0.1', state_port))
                                    result = command(client, 'status')
                                    self.assertEqual(result.returncode, 0, result.stderr)
                                    if all(' ' + field in result.stdout for field in expected):
                                        break
                                    self.assertLess(time.monotonic(), deadline, result.stdout)
                                    time.sleep(.02)
                        self.assertEqual(command(clients[-1], 'quit').returncode, 0)
                        self.assertEqual(process.wait(timeout=5), 0)
                        log.seek(0)
                        self.assertRegex(log.read(), r'robot state: \d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3} .*battery_level=.*battery_current=.*charger_in1=.*charger_in2=')
                    finally:
                        if process.poll() is None:
                            process.terminate()
                            process.wait(timeout=5)

    def test_bpx_pro_run_and_trotrun_commands(self):
        channel = robot_features.RobotFeaturesTest.channel
        channel.send(((1, 'BPX-Pro-SN'), 0))
        self.assertTrue(channel.poll(3), 'peer configuration timed out')
        self.assertIsNone(channel.recv())
        servers = [[sys.executable, str(ROOT / 'example/motion_level_control_server_example.py')]]
        clients = [[sys.executable, str(ROOT / 'example/motion_level_control_client_example.py')]]
        if os.environ.get('MIRRORME_ROBOT_SDK_EXAMPLE_BUILD'):
            build = Path(os.environ['MIRRORME_ROBOT_SDK_EXAMPLE_BUILD'])
            servers.append([str(build / 'motion_level_control_server_example')])
            clients.append([str(build / 'motion_level_control_client_example')])
        try:
            for server in servers:
                with self.subTest(server=server[-1]):
                    local_help = subprocess.run(server + ['--help'], capture_output=True, text=True, timeout=5)
                    self.assertEqual(local_help.returncode, 0, local_help.stderr)
                    self.assertRegex(local_help.stdout, r'\brun\b')
                    self.assertNotRegex(local_help.stdout, r'\brunning\b')
                    with socket.socket() as reservation:
                        reservation.bind(('127.0.0.1', 0))
                        port = reservation.getsockname()[1]
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
                        reservation.bind(('127.0.0.1', 0))
                        state_port = reservation.getsockname()[1]
                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp, tempfile.TemporaryFile(mode='w+') as log:
                        udp.bind(('127.0.0.1', 9527))
                        udp.settimeout(.1)
                        process = subprocess.Popen(server + ['--robot-ip', '127.0.0.1', '--state-port', str(state_port),
                                                           '--listen-port', str(port)], stdout=log, stderr=log)
                        try:
                            deadline = time.monotonic() + 8
                            while True:
                                try:
                                    with socket.create_connection(('127.0.0.1', port), timeout=.2) as conn:
                                        self.assertIn(b'OK', conn.recv(1024))
                                    break
                                except OSError:
                                    self.assertIsNone(process.poll())
                                    self.assertLess(time.monotonic(), deadline)
                                    time.sleep(.02)

                            def command(client, *words):
                                return subprocess.run(client + ['--server-port', str(port), *words],
                                                      capture_output=True, text=True, timeout=5)

                            def await_gait(expected):
                                deadline = time.monotonic() + 3
                                while time.monotonic() < deadline:
                                    try:
                                        packet = udp.recv(1024)
                                    except socket.timeout:
                                        continue
                                    if len(packet) == 56 and packet[4] == 6 and packet[5] == expected:
                                        self.assertEqual(packet[34], 0)
                                        self.assertEqual(struct.unpack_from('<6f', packet, 8), (0,) * 6)
                                        return
                                self.fail(f'missing motion UDP gait {expected}')

                            for client in clients:
                                with self.subTest(client=client[-1]):
                                    deadline = time.monotonic() + 5
                                    while True:
                                        status = command(client, 'status')
                                        if 'connected=1' in status.stdout:
                                            break
                                        self.assertLess(time.monotonic(), deadline, status.stdout + status.stderr)
                                        time.sleep(.02)
                                    self.assertIn('model=BPX-Pro', status.stdout)
                                    # Observe a new safe baseline after connection readiness;
                                    # a queued packet from before acceptance is insufficient.
                                    if client is clients[0]:
                                        udp.setblocking(False)
                                        try:
                                            while True:
                                                udp.recv(1024)
                                        except BlockingIOError:
                                            pass
                                        finally:
                                            udp.settimeout(.1)
                                        await_gait(0)
                                    for name, expected in (('run', 8), ('trotrun', 15)):
                                        result = command(client, 'gait', name)
                                        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                                        self.assertIn('OK gait command sent', result.stdout)
                                        await_gait(expected)
                                    rejected = command(client, 'gait', 'running')
                                    self.assertNotEqual(rejected.returncode, 0)
                                    self.assertIn('ERR unknown gait', rejected.stdout)
                            self.assertEqual(command(clients[-1], 'quit').returncode, 0)
                            self.assertEqual(process.wait(timeout=5), 0)
                        finally:
                            if process.poll() is None:
                                process.terminate()
                                try:
                                    process.wait(timeout=5)
                                except subprocess.TimeoutExpired:
                                    process.kill()
                                    process.wait(timeout=5)
        finally:
            channel.send(((0, 'original-SN'), 0))
            self.assertTrue(channel.poll(3), 'peer restore timed out')
            self.assertIsNone(channel.recv())

    def test_bpw_rough_and_unsupported_commands(self):
        channel = robot_features.RobotFeaturesTest.channel
        channel.send(((3, 'BPW-SN'), 0))
        self.assertTrue(channel.poll(3))
        self.assertIsNone(channel.recv())
        channel.send({'capabilities': joint_profile()})
        self.assertTrue(channel.poll(3))
        self.assertIsNone(channel.recv())
        servers = [[sys.executable, str(ROOT / 'example/motion_level_control_server_example.py')]]
        clients = [[sys.executable, str(ROOT / 'example/motion_level_control_client_example.py')]]
        if os.environ.get('MIRRORME_ROBOT_SDK_EXAMPLE_BUILD'):
            build = Path(os.environ['MIRRORME_ROBOT_SDK_EXAMPLE_BUILD'])
            servers.append([str(build / 'motion_level_control_server_example')])
            clients.append([str(build / 'motion_level_control_client_example')])
        feedback_sequence = 1
        try:
            for server in servers:
                with socket.socket() as reservation:
                    reservation.bind(('127.0.0.1', 0))
                    port = reservation.getsockname()[1]
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
                    reservation.bind(('127.0.0.1', 0))
                    state_port = reservation.getsockname()[1]
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp, tempfile.TemporaryFile(mode='w+') as log:
                    udp.bind(('127.0.0.1', 9527))
                    udp.settimeout(.1)
                    process = subprocess.Popen(server + ['--robot-ip', '127.0.0.1', '--state-port', str(state_port),
                                               '--listen-port', str(port)], stdout=log, stderr=log)
                    try:
                        deadline = time.monotonic() + 8
                        while True:
                            try:
                                with socket.create_connection(('127.0.0.1', port), timeout=.2) as conn:
                                    self.assertIn(b'OK', conn.recv(1024))
                                break
                            except OSError:
                                self.assertIsNone(process.poll())
                                self.assertLess(time.monotonic(), deadline)
                                time.sleep(.02)
                        for client in clients:
                            def command(*words):
                                return subprocess.run(client + ['--server-port', str(port), *words],
                                                      capture_output=True, text=True, timeout=5)
                            deadline = time.monotonic() + 3
                            while 'connected=1' not in command('status').stdout:
                                self.assertLess(time.monotonic(), deadline)
                                time.sleep(.01)
                            # A listening example server is not proof that its
                            # asynchronous SDK sender has sent the safe baseline.
                            udp.setblocking(False)
                            try:
                                while True:
                                    udp.recv(1024)
                            except BlockingIOError:
                                pass
                            finally:
                                udp.settimeout(.1)
                            deadline = time.monotonic() + 3
                            while True:
                                self.assertLess(time.monotonic(), deadline)
                                try:
                                    baseline = udp.recv(1024)
                                except socket.timeout:
                                    continue
                                self.assertEqual(len(baseline), 56)
                                if (baseline[4] == 6 and baseline[5] == 0 and
                                        struct.unpack_from('<6f', baseline, 8) == (0,) * 6):
                                    break
                            result = command('gait', 'rough')
                            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                            deadline = time.monotonic() + 3
                            while True:
                                self.assertLess(time.monotonic(), deadline)
                                try:
                                    packet = udp.recv(1024)
                                except socket.timeout:
                                    continue
                                self.assertEqual(len(packet), 56)
                                if packet[5] == 16:
                                    break
                            self.assertEqual(packet[34], 0)
                            for words in [('gait', 'run'), ('gait', 'trotrun'),
                                          ('gait', 'gap'), ('gait', 'triple'), ('gait', 'walk_period'),
                                          ('jump', 'up'), ('charging-sit',), ('tumbler',), ('zero',)]:
                                rejected = command(*words)
                                self.assertNotEqual(rejected.returncode, 0)
                                self.assertIn('unsupported on BPW', rejected.stdout)
                            payload = struct.pack('<5B3x3f', 6, 16, 1, 0, 0, 1.5, 1, 1)
                            deadline = time.monotonic() + 3
                            while True:
                                udp.sendto(struct.pack('<HHI', 0x10, 20, 100) + payload, ('127.0.0.1', state_port))
                                channel.send('capability_sessions')
                                self.assertTrue(channel.poll(3))
                                sessions = channel.recv()
                                active = [item for item in sessions if item['port'] == state_port and item['enabled']]
                                if active:
                                    state = max(active, key=lambda item: item['token'])
                                    # No E1 packet is sent: both real servers must
                                    # derive joint_count from the fresh E2 snapshot.
                                    records = [(joint_id, float(index), .5, -.5)
                                               for index, (joint_id, _, _) in enumerate(joint_profile()['joints'])]
                                    for packet in joint_states(state['token'], 7, records, sequence=feedback_sequence):
                                        udp.sendto(packet, ('127.0.0.1', state_port))
                                    feedback_sequence += 1
                                status = command('status')
                                if 'gait=16(Rough)' in status.stdout and 'joint_count=16' in status.stdout:
                                    break
                                self.assertLess(time.monotonic(), deadline, status.stdout)
                            self.assertIn('model=BPW', status.stdout)
                            self.assertNotIn('cap_protocol=', status.stdout)
                            self.assertIn('joint_count=16', status.stdout)
                            self.assertEqual(command('gait', 'walk').returncode, 0)
                        self.assertEqual(command('quit').returncode, 0)
                        self.assertEqual(process.wait(timeout=5), 0)
                    finally:
                        if process.poll() is None:
                            process.terminate()
                            process.wait(timeout=5)
        finally:
            channel.send(((0, 'original-SN'), 0))
            self.assertTrue(channel.poll(3))
            self.assertIsNone(channel.recv())

"""Run both RequestRobotState CLIs against independent TCP/UDP fixtures.

These are behavior checks of the shipped examples, not source-text assertions.
All peers and packets stay on loopback; no hardware is involved.
"""
import ast
from contextlib import contextmanager
import os
from pathlib import Path
import re
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest

from capability_wire import joint_profile, joint_states, temperatures
import test_python_robot_features as robot_features

ROOT = Path(__file__).resolve().parents[1]


def packet(head, payload, timestamp):
    return struct.pack('<HHI', head, len(payload), timestamp) + payload


def fault_packet(entries, timestamp=106, truncated=False):
    payload = bytearray(520)
    struct.pack_into('<IB3x', payload, 0, len(entries), truncated)
    for index, (code, occurred) in enumerate(entries):
        struct.pack_into('<QQ', payload, 8 + 16 * index, code, occurred)
    return packet(0xF0, payload, timestamp)


class ExampleProcess:
    def __init__(self, test, command, port, directory):
        self.test = test
        self.output_path = Path(directory) / 'stdout.log'
        self.errors_path = Path(directory) / 'stderr.log'
        self.output_file = self.output_path.open('w')
        self.errors_file = self.errors_path.open('w')
        self.process = subprocess.Popen(command + ['--robot-ip', '127.0.0.1',
                                                   '--state-port', str(port)],
                                        stdout=self.output_file, stderr=self.errors_file)

    @property
    def output(self):
        return self.output_path.read_text()

    @property
    def errors(self):
        return self.errors_path.read_text()

    @staticmethod
    def frames(output):
        return re.split(r'(?m)^robot state: [^\n]+\n', output)[1:]

    def wait_frame(self, predicate, send=None, start=0, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.test.assertIsNone(self.process.poll(), self.output + self.errors)
            if send:
                send()
            for frame in reversed(self.frames(self.output[start:])):
                if predicate(frame):
                    return frame
            time.sleep(.03)
        self.test.fail('Expected example frame was not observed:\n' + self.output + '\n' + self.errors)

    def stop(self, verify=True):
        if self.process.poll() is None:
            self.process.terminate()  # SIGTERM must use the example's graceful path.
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=3)
            if verify:
                self.test.fail('example did not exit cleanly after SIGTERM')
        self.output_file.close()
        self.errors_file.close()
        if verify:
            # Windows terminate() is TerminateProcess, not a delivered SIGTERM.
            if os.name == 'posix':
                self.test.assertEqual(self.process.returncode, 0, self.output + self.errors)
                self.test.assertIn('connection: stopped', self.output)
            self.test.assertNotIn('Traceback', self.errors)


class CompleteRequestExamplesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        robot_features.RobotFeaturesTest.setUpClass()
        cls.channel = robot_features.RobotFeaturesTest.channel

    @classmethod
    def tearDownClass(cls):
        robot_features.RobotFeaturesTest.tearDownClass()

    def ask(self, message):
        self.channel.send(message)
        self.assertTrue(self.channel.poll(3), 'fake peer control timed out')
        return self.channel.recv()

    def configure(self, model=0, config=None):
        self.assertIsNone(self.ask(((model, 'complete-state-SN'), 0)))
        self.assertIsNone(self.ask({'pause_accept': False}))
        if config is not None:
            self.assertIsNone(self.ask({'capabilities': config}))

    def setUp(self):
        self.configure()
        self.addCleanup(self.configure)

    @staticmethod
    def commands():
        commands = [[sys.executable, '-u', str(ROOT / 'example/request_robot_state_example.py')]]
        build = os.environ.get('MIRRORME_ROBOT_SDK_EXAMPLE_BUILD')
        if build:
            executable = Path(build) / 'request_robot_state_example'
            if not executable.is_file():
                raise AssertionError(str(executable))
            commands.append([str(executable)])
        return commands

    @contextmanager
    def example(self, command):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        with tempfile.TemporaryDirectory() as directory:
            example = ExampleProcess(self, command, port, directory)
            try:
                yield example, port
            except BaseException:
                example.stop(verify=False)
                raise
            else:
                example.stop()

    @staticmethod
    def send(port, packets):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for value in packets:
                sender.sendto(value, ('127.0.0.1', port))

    def active_session(self, port):
        sessions = self.ask('capability_sessions')
        active = [value for value in sessions if value['port'] == port and value['enabled']]
        return max(active, key=lambda value: value['token']) if active else None

    def values(self, frame, name, count):
        match = re.search(r'(?m)^\s*' + name + r' count=(\d+) values=(\[[^\n]*\])$', frame)
        self.assertIsNotNone(match, frame)
        self.assertEqual(int(match[1]), count, frame)
        values = ast.literal_eval(match[2])
        self.assertEqual(len(values), count)
        return values

    def tuple_values(self, frame, name):
        match = re.search(r'(?m)(?<!\S)' + re.escape(name) + r'=(\([^()\n]*\))(?=\s|$)', frame)
        self.assertIsNotNone(match, frame)
        return ast.literal_eval(match[1])

    def timestamps(self, frame):
        match = re.search(r'(?m)^\s*timestamps_ms (.+)$', frame)
        self.assertIsNotNone(match, frame)
        return dict(re.findall(r'(\w+)=(<[^>]+>|\d+)', match[1]))

    @staticmethod
    def basic_packets(offset=0, include_odom=True, entries=None):
        if entries is None:
            entries = [(0x100000F205000003, 1712345678901),
                       (0x100000F102070005, 1712345679999)]
        result = [
            packet(0x200, struct.pack('<13f', .125, -.25, .5, .1, .2, .3, .9,
                                      1.25, 2.5, -9.75, .5, -.75, 1.125), 102 + offset),
            packet(0x10, struct.pack('<5B3x3f', 6, 16, 1, 8, 254, 1.5, .5, 2.0), 104 + offset),
            packet(1, struct.pack('<BB2xf24b', 64, 7, 1.25,
                                   *range(12), *range(20, 32)), 105 + offset),
            fault_packet(entries, 106 + offset),
        ]
        if include_odom:
            result.append(packet(0x50, struct.pack('<13f', *range(1, 14)), 103 + offset))
        return result

    def assert_basic_values(self, frame, odom='103', offset=0):
        for name, expected in [('rpy', (.125, -.25, .5)), ('quat', (.1, .2, .3, .9)),
                               ('acc', (1.25, 2.5, -9.75)), ('omega', (.5, -.75, 1.125)),
                               ('max_vel', (1.5, .5, 2.0))]:
            actual = self.tuple_values(frame, name)
            self.assertEqual(len(actual), len(expected))
            for value, wanted in zip(actual, expected):
                self.assertAlmostEqual(value, wanted, places=5, msg=frame)
        for name, value in [('motion_state', 6), ('gait', 16), ('sub_gait', -2),
                            ('last_state', 1), ('last_gait', 8)]:
            self.assertRegex(frame, r'\b' + name + '=' + str(value) + r'(?:\s|$)')
        stamps = self.timestamps(frame)
        self.assertEqual({key: stamps[key] for key in ('imu', 'odometry', 'motion', 'battery', 'fault')},
                         {'imu': str(102 + offset), 'odometry': odom, 'motion': str(104 + offset),
                          'battery': str(105 + offset), 'fault': str(106 + offset)})
        for value in ('battery_level=64%', 'battery_current=1.250', 'charger_in1=1',
                      'charger_in2=1', 'hard_estop=1', 'fault_details count=2 truncated=false',
                      'fault index=0 code=0x100000f205000003 occurred_t_ms=1712345678901',
                      'fault index=1 code=0x100000f102070005 occurred_t_ms=1712345679999'):
            self.assertIn(value, frame)

    def test_legacy_bpx_complete_feedback_reconnect_and_sigterm(self):
        for command in self.commands():
            with self.subTest(command=command[0]):
                self.configure()
                with self.example(command) as (example, port):
                    joint = packet(0x1000, struct.pack('<36f',
                        *[index + .25 for index in range(12)],
                        *[index + 10.5 for index in range(12)],
                        *[-index - .75 for index in range(12)]), 101)
                    def send():
                        self.send(port, self.basic_packets() + [joint])
                    frame = example.wait_frame(lambda value: 'fault index=1' in value and
                                               'joint_vel count=12' in value and 'timestamps_ms joint=101 ' in value,
                                               send=send)
                    self.assert_basic_values(frame)
                    self.assertEqual(self.values(frame, 'joint_pos', 12)[-1], 11.25)
                    self.assertEqual(self.values(frame, 'joint_vel', 12)[-1], 21.5)
                    self.assertEqual(self.values(frame, 'joint_tau', 12)[-1], -11.75)
                    self.assertEqual(self.values(frame, 'motor_temperature', 12)[-1], 71)
                    self.assertEqual(self.values(frame, 'driver_temperature', 12)[-1], 91)
                    self.assertIn('connected=true', frame)
                    self.assertIn('connection: connecting', example.output)
                    self.assertIn('connection: connected', example.output)
                    self.assertIn('robot version (cached): 0.0.0', example.output)
                    self.assertIn('robot identity: serial_number=complete-state-SN model=BPX (1)', example.output)
                    self.assertEqual(self.tuple_values(frame, 'velocity_body'), (1, 2, 3))
                    self.assertEqual(self.tuple_values(frame, 'leg_odom.position'), (4, 5, 6))
                    self.assertEqual(self.tuple_values(frame, 'orientation'), (7, 8, 9, 10))
                    self.assertEqual(self.tuple_values(frame, 'angular_velocity'), (11, 12, 13))
                    start = len(example.output)
                    self.assertIsNone(self.ask({'pause_accept': True}))
                    self.assertIsNone(self.ask('drop_clients'))
                    disconnected = example.wait_frame(lambda value: 'connected=false' in value and
                        'telemetry=<unavailable: disconnected>' in value, start=start)
                    for field in ('joint_pos', 'rpy=', 'motion_state=', 'battery_level=', 'timestamps_ms', 'fault_details'):
                        self.assertNotIn(field, disconnected)
                    # Hold the outage longer than the examples' 200 ms observation interval.
                    time.sleep(.25)
                    self.assertIn('connection: disconnected', example.output[start:])
                    self.assertIn('retrying', example.output[start:])
                    self.assertIsNone(self.ask({'pause_accept': False}))
                    replacement = packet(0x1000, struct.pack('<36f', *range(36)), 201)
                    def send_reconnected():
                        self.send(port, self.basic_packets(offset=100) + [replacement])
                    frame = example.wait_frame(lambda value: 'connected=true' in value and
                        'timestamps_ms joint=201 ' in value and 'fault index=1' in value,
                        send=send_reconnected, start=start)
                    self.assert_basic_values(frame, odom='203', offset=100)
                    self.assertEqual(self.values(frame, 'joint_pos', 12)[-1], 11)
                    self.assertIn('connection: reconnected', example.output[start:])
                    self.assertIn('robot version (cached):', example.output[start:])
                    self.assertIn('robot identity: serial_number=complete-state-SN', example.output[start:])

    def test_bpw_sixteen_axes_independent_temperatures_and_partial_dimensions(self):
        config = joint_profile()
        for command in self.commands():
            with self.subTest(command=command[0]):
                self.configure(3, config)
                with self.example(command) as (example, port):
                    sequence = 0
                    motor_missing, driver_missing, velocity_missing = [False], [False], [False]
                    def send():
                        nonlocal sequence
                        session = self.active_session(port)
                        if session is None:
                            return
                        sequence += 1
                        state = [(joint_id, float(index), None if velocity_missing[0] and index == 7 else index + .5,
                                  -float(index)) for index, (joint_id, _, _) in enumerate(config['joints'])]
                        temp = [(joint_id, None if motor_missing[0] and index == 3 else float(index),
                                 None if driver_missing[0] and index == 5 else 40.5 + index)
                                for index, (joint_id, _, _) in enumerate(config['joints'])]
                        packets = self.basic_packets(include_odom=False)
                        packets += joint_states(session['token'], 7, state, sequence, timestamp=0x10000004D)
                        packets += temperatures(session['token'], 7, temp, sequence, timestamp=0x100000063)
                        self.send(port, packets)
                    frame = example.wait_frame(lambda value: 'driver_temperature count=16' in value and
                        'fault index=1' in value and 'timestamps_ms joint=77 ' in value, send=send)
                    self.assert_basic_values(frame, odom='<unsupported>')
                    self.assertEqual(self.values(frame, 'joint_pos', 16), list(range(16)))
                    self.assertEqual(self.values(frame, 'joint_vel', 16)[-1], 15.5)
                    self.assertEqual(self.values(frame, 'joint_tau', 16)[-1], -15)
                    self.assertEqual(self.values(frame, 'motor_temperature', 16)[-1], 15)
                    self.assertEqual(self.values(frame, 'driver_temperature', 16)[-1], 55.5)
                    self.assertIn('timestamp_ms=4294967373', frame)
                    self.assertIn('timestamp_ms=4294967395', frame)
                    self.assertIn('leg_odom=<unsupported>', frame)
                    start = len(example.output)
                    motor_missing[0] = True
                    frame = example.wait_frame(lambda value: 'motor_temperature=<unavailable>' in value and
                        'driver_temperature count=16' in value and
                        'joint_id=109 motor_celsius=<unavailable> driver_celsius=43.5' in value,
                        send=send, start=start)
                    self.assertEqual(self.values(frame, 'driver_temperature', 16)[-1], 55.5)
                    self.assertRegex(frame, r'joint_id=109 motor_celsius=<unavailable> driver_celsius=43\.5')
                    start = len(example.output)
                    motor_missing[0], driver_missing[0], velocity_missing[0] = False, True, True
                    frame = example.wait_frame(lambda value: 'motor_temperature count=16' in value and
                        'driver_temperature=<unavailable>' in value and 'joint_vel=<unavailable>' in value,
                        send=send, start=start)
                    self.assertEqual(self.values(frame, 'motor_temperature', 16)[-1], 15)
                    self.assertEqual(self.values(frame, 'joint_tau', 16)[-1], -15)
                    self.assertIn('joint_id=121 position=7 velocity=<unavailable> torque=-7', frame)
                    self.assertNotIn('unsupported on BPW', example.errors)

    def test_future_sixty_four_axes_unknown_readings_and_all_fault_entries(self):
        config = joint_profile(64, 99, revision=19)
        entries = [(0x100000F102000001 + index, 1712345600000 + index) for index in range(32)]
        for command in self.commands():
            with self.subTest(command=command[0]):
                self.configure(99, config)
                with self.example(command) as (example, port):
                    empty = example.wait_frame(lambda value: 'connected=true' in value and
                        'joint_states=<unavailable>' in value and 'timestamps_ms' in value and
                        'fault_details=<unavailable>' in value)
                    for name in ('joint_pos', 'joint_vel', 'joint_tau', 'rpy', 'quat', 'acc', 'omega',
                                 'motor_temperature', 'driver_temperature', 'max_vel', 'motion_state',
                                 'gait', 'sub_gait', 'last_state', 'last_gait', 'fault_details'):
                        self.assertIn(name + '=<unavailable>', empty)
                    self.assertEqual(set(self.timestamps(empty).values()), {'<unavailable>'})
                    sequence = 0
                    def send():
                        nonlocal sequence
                        session = self.active_session(port)
                        if session is None:
                            return
                        sequence += 1
                        states = [(joint_id, index + .25, index + .5, index + .75)
                                  for index, (joint_id, _, _) in enumerate(config['joints'])]
                        temp = [(joint_id, float(index), 20.5 + index)
                                for index, (joint_id, _, _) in enumerate(config['joints'])]
                        values = joint_states(session['token'], 19, states, sequence, timestamp=9876543210123)
                        values += temperatures(session['token'], 19, temp, sequence, timestamp=9876543210999)
                        values += [fault_packet(entries, 106, truncated=True)]
                        self.send(port, values)
                    frame = example.wait_frame(lambda value: 'joint_pos count=64' in value and
                        'driver_temperature count=64' in value and 'fault index=31' in value, send=send)
                    for name, last in [('joint_pos', 63.25), ('joint_vel', 63.5), ('joint_tau', 63.75),
                                       ('motor_temperature', 63), ('driver_temperature', 83.5)]:
                        self.assertEqual(self.values(frame, name, 64)[-1], last)
                    self.assertIn('joint_id=289 position=63.25 velocity=63.5 torque=63.75', frame)
                    self.assertIn('fault_details count=32 truncated=true', frame)
                    for index, (code, occurred) in enumerate(entries):
                        self.assertIn(f'fault index={index} code=0x{code:x} occurred_t_ms={occurred}', frame)
                    self.assertNotIn('fault index=32', frame)
                    self.assertEqual(self.timestamps(frame)['joint'], str(9876543210123 & 0xFFFFFFFF))
                    self.assertIn('timestamp_ms=9876543210123', frame)
                    self.assertIn('model=Unknown (0)', example.output)


if __name__ == '__main__':
    unittest.main()

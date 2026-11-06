"""Exercise autonomous C++/Python motion demos over loopback, without hardware."""
from contextlib import contextmanager
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

ROOT = Path(__file__).resolve().parents[1]


class Demo:
    def __init__(self, test, command, directory):
        self.test = test
        self.output_path = Path(directory) / 'stdout.log'
        self.error_path = Path(directory) / 'stderr.log'
        self.output_file = self.output_path.open('w')
        self.error_file = self.error_path.open('w')
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver.bind(('127.0.0.1', 9527))
        self.receiver.setblocking(False)
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(('127.0.0.1', 0))
            self.port = reservation.getsockname()[1]
        self.state, self.gait, self.timestamp = 0, 16, 100
        self.upload = False
        self.freeze_timestamp = False
        self.commands = []
        self.process = subprocess.Popen(command + ['--robot-ip', '127.0.0.1',
                                                   '--state-port', str(self.port)],
                                        stdout=self.output_file, stderr=self.error_file)

    @property
    def output(self):
        return self.output_path.read_text()

    @property
    def errors(self):
        return self.error_path.read_text()

    def pump(self):
        if self.upload:
            if not self.freeze_timestamp:
                self.timestamp = (self.timestamp + 1) & 0xffffffff
            payload = struct.pack('<5B3x3f', self.state, self.gait, 1, 0, 0, 1.5, 1.0, 1.0)
            self.sender.sendto(struct.pack('<HHI', 0x10, len(payload), self.timestamp) + payload,
                               ('127.0.0.1', self.port))
        self.drain()
        time.sleep(.02)

    def drain(self):
        while True:
            try:
                packet = self.receiver.recv(1024)
            except BlockingIOError:
                break
            self.test.assertEqual(len(packet), 56)
            self.commands.append({'time': time.monotonic(), 'mode': packet[4], 'gait': packet[5],
                                  'axes': struct.unpack_from('<6f', packet, 8),
                                  'si_units': packet[32], 'zero': packet[33],
                                  'version': struct.unpack_from('<I', packet, 40)[0]})

    def run_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.test.assertIsNone(self.process.poll(), self.output + self.errors)
            self.pump()

    def wait(self, predicate, timeout=5, allow_exit=False):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if predicate():
                if self.process.poll() is not None:
                    self.drain()
                return
            if not allow_exit:
                self.test.assertIsNone(self.process.poll(), self.output + self.errors)
        self.test.fail('Expected demo behavior did not arrive:\n' + self.output + self.errors)

    def moving(self):
        return [value for value in self.commands if any(value['axes'])]

    def wait_stand(self):
        self.upload = True
        self.wait(lambda: any(value['mode'] == 1 for value in self.commands))

    def wait_walk(self):
        self.state = 6
        self.wait(lambda: any(value['mode'] == 6 for value in self.after_stand()))

    def after_stand(self):
        for index, value in enumerate(self.commands):
            if value['mode'] == 1:
                return self.commands[index + 1:]
        return []

    def wait_yaw(self):
        self.wait_stand()
        self.wait_walk()
        self.gait = 0
        self.wait(lambda: bool(self.moving()))

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
                self.test.fail('Demo did not terminate')
        self.receiver.close()
        self.sender.close()
        self.output_file.close()
        self.error_file.close()


class MotionLevelExamplesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        robot_features.RobotFeaturesTest.setUpClass()
        cls.channel = robot_features.RobotFeaturesTest.channel

    @classmethod
    def tearDownClass(cls):
        robot_features.RobotFeaturesTest.tearDownClass()

    def ask(self, message):
        self.channel.send(message)
        self.assertTrue(self.channel.poll(3), 'Peer control timed out')
        return self.channel.recv()

    def configure(self, model=3):
        self.assertIsNone(self.ask(((model, 'motion-demo-SN'), 0)))
        self.assertIsNone(self.ask({'pause_accept': False}))

    @staticmethod
    def commands():
        commands = [[sys.executable, '-u', str(ROOT / 'example/motion_level_control_example.py')]]
        build = os.environ.get('MIRRORME_ROBOT_SDK_EXAMPLE_BUILD')
        if build:
            executable = Path(build) / 'motion_level_control_example'
            if not executable.is_file():
                raise AssertionError(str(executable))
            commands.append([str(executable)])
        return commands

    @contextmanager
    def example(self, command, model=3):
        self.configure(model)
        with tempfile.TemporaryDirectory() as directory:
            demo = Demo(self, command, directory)
            try:
                yield demo
            finally:
                demo.close()
                self.configure()

    def assert_walk_only(self, demo):
        self.assertTrue(demo.commands, demo.output + demo.errors)
        self.assertTrue(all(value['gait'] == 0 for value in demo.commands), demo.commands)
        self.assertNotIn('left-flip', demo.output)
        self.assertNotIn('Traceback', demo.errors)

    def test_bpw_waits_for_initial_state_fresh_walk_and_completed_sit(self):
        for command in self.commands():
            with self.subTest(command=command), self.example(command) as demo:
                # TCP acceptance alone must not trigger calibration or stand-up.
                demo.wait(lambda: any(session['port'] == demo.port
                                     for session in self.ask('capability_sessions')))
                demo.run_for(.4)
                self.assertFalse(demo.moving())
                self.assertFalse(any(value['mode'] == 1 or value['zero'] for value in demo.commands))
                demo.wait_stand()
                demo.run_for(.3)
                self.assertFalse(demo.moving())
                demo.wait_walk()
                demo.run_for(.15)
                self.assertFalse(demo.moving())
                # Matching state/gait with an unchanged timestamp is cached feedback.
                demo.freeze_timestamp = True
                demo.wait(lambda: any('gait=16 ' in line and
                                      f'motion_timestamp_ms={demo.timestamp}' in line
                                      for line in demo.output.splitlines()), timeout=.5)
                demo.gait = 0
                demo.run_for(.35)
                self.assertFalse(demo.moving())
                demo.freeze_timestamp = False
                demo.wait(lambda: bool(demo.moving()))
                first = demo.moving()[0]['time']
                demo.wait(lambda: any(value['time'] > first and not any(value['axes'])
                                      for value in demo.commands), timeout=7)
                stopped = next(value['time'] for value in demo.commands
                               if value['time'] > first and not any(value['axes']))
                self.assertGreaterEqual(stopped - first, 2.8)
                self.assertLess(stopped - first, 3.5)
                for value in demo.moving():
                    self.assertEqual(value['axes'], (0.0, 0.0, 1.0, 0.0, 0.0, 0.0))
                    self.assertEqual(value['si_units'], 1)
                demo.wait(lambda: any(value['mode'] == 3 for value in demo.commands), timeout=4)
                first_sit = next(value['time'] for value in demo.commands if value['mode'] == 3)
                self.assertGreaterEqual(first_sit - stopped, 1.8)
                # The old example incorrectly exited after a fixed five seconds.
                demo.state = 3
                demo.run_for(5.2)
                demo.state = 0
                demo.wait(lambda: demo.process.poll() is not None, allow_exit=True)
                self.assertEqual(demo.process.returncode, 0, demo.output + demo.errors)
                self.assertTrue(all(value['zero'] == 0 for value in demo.commands))
                self.assertIn('BPW: skip zero-position command', demo.output)
                sit_index = next(index for index, value in enumerate(demo.commands) if value['mode'] == 3)
                last_sit_version = None
                for value in demo.commands[sit_index:]:
                    if value['mode'] == 3:
                        last_sit_version = value['version']
                    else:
                        # disconnect() itself emits a neutral SDK baseline with the
                        # same business version. The demo must not add a new move.
                        self.assertEqual(value['mode'], 6)
                        self.assertEqual(value['version'], last_sit_version)
                        self.assertFalse(any(value['axes']))
                self.assert_walk_only(demo)

    def test_bpx_and_pro_preserve_calibration_and_graceful_stop(self):
        for command in self.commands():
            for model in (0, 1):
                with self.subTest(command=command, model=model), self.example(command, model) as demo:
                    demo.wait(lambda: any(session['port'] == demo.port
                                         for session in self.ask('capability_sessions')))
                    demo.run_for(.3)
                    self.assertFalse(any(value['mode'] == 1 or value['zero'] for value in demo.commands))
                    demo.wait_stand()
                    self.assertTrue(any(value['zero'] for value in demo.commands), demo.output)
                    self.assertIn('send zero-position command', demo.output)
                    demo.process.terminate()
                    demo.wait(lambda: demo.process.poll() is not None, allow_exit=True)
                    if os.name == 'posix':
                        self.assertEqual(demo.process.returncode, 0, demo.output + demo.errors)
                        self.assertIn('connection: stopped', demo.output)
                    self.assertFalse(demo.moving())
                    self.assert_walk_only(demo)

    def test_walk_timeout_requires_feedback_even_while_packets_are_fresh(self):
        for command in self.commands():
            with self.subTest(command=command), self.example(command) as demo:
                demo.wait_stand()
                demo.wait_walk()
                # Fresh motion packets keep arriving, but gait never acknowledges Walk.
                demo.wait(lambda: demo.process.poll() is not None, timeout=12, allow_exit=True)
                self.assertEqual(demo.process.returncode, 1, demo.output + demo.errors)
                self.assertIn('timeout', demo.errors.lower())
                self.assertIn('walk', demo.errors.lower())
                self.assertFalse(demo.moving())
                self.assertFalse(any(value['mode'] == 3 for value in demo.commands))
                self.assert_walk_only(demo)

    def test_motion_staleness_and_connection_loss_stop_without_resuming(self):
        for command in self.commands():
            for failure in ('stale', 'disconnect'):
                with self.subTest(command=command, failure=failure), self.example(command) as demo:
                    demo.wait_yaw()
                    began = time.monotonic()
                    if failure == 'stale':
                        demo.freeze_timestamp = True
                    else:
                        self.assertIsNone(self.ask({'pause_accept': True}))
                        self.assertIsNone(self.ask('drop_clients'))
                    demo.wait(lambda: demo.process.poll() is not None, timeout=3, allow_exit=True)
                    self.assertEqual(demo.process.returncode, 1, demo.output + demo.errors)
                    self.assertLess(time.monotonic() - began, 3)
                    if failure == 'stale':
                        self.assertIn('motion feedback timeout', demo.errors.lower())
                        self.assertFalse(any(demo.commands[-1]['axes']))
                    else:
                        self.assertIn('connection', demo.errors.lower())
                    self.assertFalse(any(value['mode'] == 3 for value in demo.commands))
                    self.assert_walk_only(demo)

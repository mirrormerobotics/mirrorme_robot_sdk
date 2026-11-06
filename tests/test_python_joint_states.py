"""Dynamic joint state APIs driven by an independent E2 wire implementation."""
import socket
import struct
import time
import unittest

import mirrorme_robot_sdk
from capability_wire import joint_profile, joint_states
import test_python_robot_features as robot_features


class DynamicJointStatesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        robot_features.RobotFeaturesTest.setUpClass()
        cls.channel = robot_features.RobotFeaturesTest.channel

    @classmethod
    def tearDownClass(cls):
        robot_features.RobotFeaturesTest.tearDownClass()

    wait = robot_features.RobotFeaturesTest.wait
    connect = robot_features.RobotFeaturesTest.connect
    configure_peer = robot_features.RobotFeaturesTest.configure_peer

    def setUp(self):
        self.configure_peer()
        self.addCleanup(self.configure_peer)

    def ask(self, message):
        self.channel.send(message)
        self.assertTrue(self.channel.poll(3))
        return self.channel.recv()

    def configure(self, config, model=3):
        self.configure_peer(model, 'dynamic-joints')
        self.assertIsNone(self.ask({'capabilities': config}))

    def session(self, port):
        return max((value for value in self.ask('capability_sessions')
                    if value['port'] == port and value['enabled']), key=lambda value: value['token'])

    @staticmethod
    def records(config):
        return [(joint_id, float(index), float(index) / 2, -float(index))
                for index, (joint_id, _, _) in enumerate(config['joints'])]

    @staticmethod
    def send(port, packets):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            for packet in packets:
                sender.sendto(packet, ('127.0.0.1', port))

    def test_sixteen_axes_each_dimension_and_timestamp_on_state_and_motion(self):
        config = joint_profile()
        self.configure(config)
        for kind in (mirrorme_robot_sdk.RequestRobotState, mirrorme_robot_sdk.MotionLevelControl):
            with self.subTest(kind=kind.__name__):
                sdk, port = self.connect(kind)
                state = self.session(port)
                self.assertEqual(state['enabled'], [(1, 1, 0), (2, 1, 0)])
                self.assertIsNone(sdk.getJointStates())
                self.assertIsNone(sdk.getJointPosition())
                records = self.records(config)
                self.send(port, joint_states(state['token'], 7, records))
                self.wait(lambda: sdk.getJointStates() is not None)
                snapshot = sdk.getJointStates()
                self.assertEqual(snapshot['timestamp_ms'], 1234567890123)
                self.assertEqual(snapshot['topology_revision'], 7)
                self.assertEqual(snapshot['sequence'], 1)
                self.assertLess(snapshot['age_ms'], 1000)
                self.assertEqual(sdk.getJointStateTimestamp(), 1234567890123 & 0xFFFFFFFF)
                self.assertEqual([joint['joint_id'] for joint in snapshot['joints']],
                                 [joint[0] for joint in config['joints']])
                for dimension, name in enumerate(('Position', 'Velocity', 'Torque'), start=1):
                    self.assertEqual(getattr(sdk, 'getJoint' + name)(), [record[dimension] for record in records])
                self.assertEqual(snapshot['joints'][0]['position'], 0.0)
                for sequence, (dimension, name, field) in enumerate(
                        ((1, 'Position', 'position'), (2, 'Velocity', 'velocity'), (3, 'Torque', 'torque')), start=2):
                    partial = list(records)
                    record = list(partial[3])
                    record[dimension] = None
                    partial[3] = tuple(record)
                    self.send(port, joint_states(state['token'], 7, partial, sequence=sequence))
                    self.wait(lambda: sdk.getJointStates()['sequence'] == sequence)
                    self.assertIsNone(sdk.getJointStates()['joints'][3][field])
                    self.assertEqual(sdk.getJointStateTimestamp(), (1234567890123 + sequence - 1) & 0xFFFFFFFF)
                    self.assertIsNone(getattr(sdk, 'getJoint' + name)())
                    for other in ('Position', 'Velocity', 'Torque'):
                        if other != name:
                            self.assertEqual(len(getattr(sdk, 'getJoint' + other)()), 16)
                sdk.disconnect()
                self.assertIsNone(sdk.getJointStates())
                self.assertIsNone(sdk.getJointStateTimestamp())

    def test_joint_control_inherits_dynamic_state_queries_on_bpx(self):
        config = joint_profile(12, 0)
        self.configure(config, model=0)
        sdk, port = self.connect(mirrorme_robot_sdk.JointLevelControl)
        state = self.session(port)
        records = self.records(config)
        self.send(port, joint_states(state['token'], 7, records))
        self.wait(lambda: sdk.getJointStates() is not None)
        self.assertEqual(sdk.getJointPosition(), [record[1] for record in records])
        self.assertEqual(sdk.getJointVelocity(), [record[2] for record in records])
        self.assertEqual(sdk.getJointTorque(), [record[3] for record in records])
        self.assertEqual(sdk.getJointStateTimestamp(), 1234567890123 & 0xFFFFFFFF)

    def test_future_145_axes_out_of_order_fragments_and_new_session(self):
        config = joint_profile(145, 0xF0010001, revision=17)
        config['joints'][-1] = (0xFFFFFFFF, 255, 'future_joint')
        self.configure(config, model=255)
        sdk, port = self.connect(mirrorme_robot_sdk.RequestRobotState)
        state = self.session(port)
        self.assertEqual(sdk.getRobotModel(), mirrorme_robot_sdk.RobotModel.Unknown)
        records = self.records(config)
        packets = joint_states(state['token'], 17, records)
        self.assertEqual(len(packets), 3)
        self.assertTrue(all(len(packet) <= 1200 for packet in packets))
        self.send(port, packets[1:][::-1])
        time.sleep(.12)
        self.assertIsNone(sdk.getJointStates(), 'incomplete snapshot must remain unpublished')
        self.send(port, packets[:1])
        self.wait(lambda: sdk.getJointPosition() is not None)
        self.assertGreaterEqual(sdk.getJointStates()['age_ms'], 100,
                                'the final fragment must not reset the age of the first fragment')
        self.assertEqual(sdk.getJointPosition(), [record[1] for record in records])
        self.assertEqual(len(sdk.getJointVelocity()), 145)
        self.assertEqual(sdk.getJointStates()['joints'][-1]['joint_id'], 0xFFFFFFFF)
        delayed = joint_states(state['token'], 17, records, sequence=2)
        self.send(port, delayed[:1])
        time.sleep(1.05)
        self.send(port, delayed[1:])
        time.sleep(.03)
        self.assertIsNone(sdk.getJointStates(), 'expired partial assembly must not publish late fragments')
        self.send(port, joint_states(state['token'], 17, records, sequence=3))
        self.wait(lambda: sdk.getJointStates() is not None)
        self.assertEqual(sdk.getJointStates()['sequence'], 3)
        sdk.disconnect()
        self.assertTrue(sdk.connect())
        self.wait(sdk.isConnected)
        replacement = self.session(port)
        self.assertNotEqual(replacement['token'], state['token'])
        self.send(port, joint_states(state['token'], 17, records, sequence=99))
        time.sleep(.03)
        self.assertIsNone(sdk.getJointStates(), 'old-session replay must not revive feedback')
        self.send(port, joint_states(replacement['token'], 17, records))
        self.wait(lambda: sdk.getJointStates() is not None)
        self.assertEqual(sdk.getJointStates()['sequence'], 1)

    def test_same_object_legacy_to_future_does_not_leak_old_timestamp(self):
        sdk, port = self.connect(mirrorme_robot_sdk.RequestRobotState)
        body = struct.pack('<36f', *range(36))
        self.send(port, [struct.pack('<HHI', 0x1000, len(body), 777) + body])
        self.wait(lambda: sdk.getJointStateTimestamp() == 777)
        sdk.disconnect()
        config = joint_profile(64, 99, revision=9)
        self.configure(config, model=99)
        self.assertTrue(sdk.connect())
        self.wait(sdk.isConnected)
        self.assertEqual(sdk.getRobotModel(), mirrorme_robot_sdk.RobotModel.Unknown)
        self.assertIsNone(sdk.getJointStates())
        self.assertIsNone(sdk.getJointStateTimestamp())
        state = self.session(port)
        self.send(port, joint_states(state['token'], 9, self.records(config)))
        self.wait(lambda: sdk.getJointStates() is not None)
        self.assertEqual(sdk.getJointStateTimestamp(), 1234567890123 & 0xFFFFFFFF)
        sdk.disconnect()
        self.assertIsNone(sdk.getJointStates())
        self.assertIsNone(sdk.getJointStateTimestamp(), 'disconnect must not reveal the old BPX timestamp')

    def test_invalid_packets_stale_data_and_legacy_cannot_override_extension(self):
        config = joint_profile()
        self.configure(config)
        sdk, port = self.connect(mirrorme_robot_sdk.RequestRobotState)
        state = self.session(port)
        records = self.records(config)
        self.send(port, joint_states(state['token'], 7, records))
        self.wait(lambda: sdk.getJointStates() is not None)
        self.send(port, joint_states(state['token'] + 1, 7, records, sequence=2))
        self.send(port, joint_states(state['token'], 8, records, sequence=2))
        invalid = list(records)
        invalid[1] = (invalid[1][0], float('nan'), 0, 0)
        self.send(port, joint_states(state['token'], 7, invalid, sequence=2))
        invalid[1] = (99999, 1, 0, 0)
        self.send(port, joint_states(state['token'], 7, invalid, sequence=2))
        time.sleep(.03)
        self.assertEqual(sdk.getJointStates()['sequence'], 1)
        # The old 12-axis packet must not be used once feature 2 is selected.
        body = struct.pack('<36f', *range(36))
        self.send(port, [struct.pack('<HHI', 0x1000, len(body), 777) + body])
        time.sleep(.55)
        self.send(port, joint_states(state['token'], 7, records))
        time.sleep(.55)
        for name in ('getJointStates', 'getJointPosition', 'getJointVelocity',
                     'getJointTorque', 'getJointStateTimestamp'):
            self.assertIsNone(getattr(sdk, name)(), name)

    def test_unsupported_feature_version_does_not_accept_e2(self):
        for model, count in ((0, 12), (3, 16)):
            with self.subTest(model=model):
                config = joint_profile(count, model)
                config['supported'] = [(2, 2, 0), (0xF0001234, 1, 0)]
                self.configure(config, model=model)
                sdk, port = self.connect(mirrorme_robot_sdk.RequestRobotState)
                self.assertTrue(all(not selected for _, selected in self.ask('capability_requests')))
                state = max((value for value in self.ask('capability_sessions') if value['port'] == port),
                            key=lambda value: value['token'])
                self.send(port, joint_states(state['token'], 7, self.records(config)))
                body = struct.pack('<36f', *range(36))
                self.send(port, [struct.pack('<HHI', 0x1000, len(body), 1234) + body])
                time.sleep(.03)
                self.assertIsNone(sdk.getJointStates())
                self.assertIsNone(sdk.getJointPosition(), 'advertised but unsupported feature 2 must not fallback')
                sdk.disconnect()

    def test_only_identified_legacy_bpx_and_pro_can_fallback(self):
        for model in (0, 1, 2, 3, 255):
            with self.subTest(model=model):
                self.configure_peer(model)
                sdk, port = self.connect(mirrorme_robot_sdk.RequestRobotState)
                body = struct.pack('<36f', *range(36))
                self.send(port, [struct.pack('<HHI', 0x1000, len(body), 1234) + body])
                if model in (0, 1):
                    self.wait(lambda: sdk.getJointStates() is not None)
                    snapshot = sdk.getJointStates()
                    self.assertEqual(snapshot['topology_revision'], 0)
                    self.assertEqual(snapshot['timestamp_ms'], 1234)
                    self.assertEqual([record['joint_id'] for record in snapshot['joints']], list(range(12)))
                    self.assertEqual(sdk.getJointPosition(), list(range(12)))
                    self.assertEqual(sdk.getJointVelocity(), list(range(12, 24)))
                    self.assertEqual(sdk.getJointTorque(), list(range(24, 36)))
                else:
                    time.sleep(.03)
                    for name in ('getJointStates', 'getJointPosition', 'getJointVelocity', 'getJointTorque'):
                        self.assertIsNone(getattr(sdk, name)(), name)
                sdk.disconnect()


if __name__ == '__main__':
    unittest.main()

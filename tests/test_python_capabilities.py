"""Exercise negotiated N-axis APIs against independent TCP/UDP wire fixtures."""
import socket
import struct
import time
import unittest

import mirrorme_robot_sdk
from capability_wire import profile, temperatures
import test_python_robot_features as robot_features


class DynamicJointTemperaturesTest(unittest.TestCase):
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

    def configure_catalog(self, value):
        self.channel.send({'capabilities': value})
        self.assertTrue(self.channel.poll(3))
        self.assertIsNone(self.channel.recv())

    def query_peer(self, command):
        self.channel.send(command)
        self.assertTrue(self.channel.poll(3))
        return self.channel.recv()

    def session(self, port):
        sessions = self.query_peer('capability_sessions')
        return max((item for item in sessions if item['port'] == port and item['enabled']),
                   key=lambda item: item['token'])

    @staticmethod
    def records(config):
        return [(joint_id, float(index), 40.5 + index)
                for index, (joint_id, _, _) in enumerate(config['joints'])]

    @staticmethod
    def send(port, packets):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            for packet in packets:
                udp.sendto(packet, ('127.0.0.1', port))

    def test_capability_negotiation_is_not_a_public_api(self):
        for kind in (mirrorme_robot_sdk.RequestRobotState, mirrorme_robot_sdk.MotionLevelControl,
                     mirrorme_robot_sdk.JointLevelControl):
            sdk = kind()
            self.assertFalse(hasattr(sdk, 'getRobotCapabilities'))
            self.assertFalse(hasattr(sdk, 'getRobotCapabilitiesValue'))
            self.assertTrue(callable(sdk.getJointStates))
            self.assertTrue(callable(sdk.getJointTemperatures))
        for name in ('RobotCapabilityId', 'RobotJointType', 'RobotCapability',
                     'RobotJointDescriptor', 'RobotCapabilities'):
            self.assertFalse(hasattr(mirrorme_robot_sdk, name))
        from mirrorme_robot_sdk import _mirrorme_robot_sdk as native
        self.assertFalse(any(name.startswith(('ROBOT_CAPABILITY_', 'ROBOT_JOINT_TYPE_'))
                             for name in dir(native)))

    def test_catalog_unknown_features_and_sixteen_axis_partial_temperatures(self):
        self.configure_peer(3, 'BPW-CAP-SN')
        config = profile()
        config['joints'][0] = (100, 1, 'joint_é')
        self.configure_catalog(config)
        sdk, port = self.connect(mirrorme_robot_sdk.RequestRobotState)
        self.assertEqual(self.query_peer('capability_requests'), [(1, []), (2, [(1, 1, 0)])])
        self.assertIsNone(sdk.getJointTemperatures())
        self.assertIsNone(sdk.getMotorTemperature())
        self.assertIsNone(sdk.getDriverTemperature())

        state = self.session(port)
        records = self.records(config)
        self.send(port, temperatures(state['token'], 7, records))
        self.wait(lambda: sdk.getJointTemperatures() is not None)
        snapshot = sdk.getJointTemperatures()
        self.assertEqual(snapshot['timestamp_ms'], 1234567890123)
        self.assertEqual(snapshot['sequence'], 1)
        self.assertEqual(snapshot['topology_revision'], 7)
        self.assertLess(snapshot['age_ms'], 3000)
        self.assertEqual([record['joint_id'] for record in snapshot['joints']],
                         [joint[0] for joint in config['joints']])
        self.assertEqual(snapshot['joints'][0]['motor_celsius'], 0.0)
        self.assertEqual(sdk.getMotorTemperature(), [record[1] for record in records])
        self.assertEqual(sdk.getDriverTemperature(), [record[2] for record in records])

        records[3] = (records[3][0], None, records[3][2])
        records[5] = (records[5][0], records[5][1], None)
        self.send(port, temperatures(state['token'], 7, records, sequence=2))
        self.wait(lambda: sdk.getJointTemperatures()['sequence'] == 2)
        snapshot = sdk.getJointTemperatures()
        self.assertIsNone(snapshot['joints'][3]['motor_celsius'])
        self.assertEqual(snapshot['joints'][3]['driver_celsius'], records[3][2])
        self.assertIsNone(snapshot['joints'][5]['driver_celsius'])
        self.assertIsNone(sdk.getMotorTemperature())
        self.assertIsNone(sdk.getDriverTemperature())
        self.send(port, temperatures(state['token'] + 1, 7, records, sequence=3))
        self.send(port, temperatures(state['token'], 8, records, sequence=3))
        invalid = self.records(config)
        invalid[2] = (invalid[2][0], float('nan'), invalid[2][2])
        self.send(port, temperatures(state['token'], 7, invalid, sequence=3))
        time.sleep(.05)
        self.assertEqual(sdk.getJointTemperatures()['sequence'], 2)
        # A legacy 12-axis packet must not replace negotiated extension feedback.
        legacy = struct.pack('<BB2xf24b', 90, 0, 0.0, *([5] * 24))
        self.send(port, [struct.pack('<HHI', 1, len(legacy), 999) + legacy])
        time.sleep(3.1)
        self.assertIsNone(sdk.getJointTemperatures())
        self.assertIsNone(sdk.getMotorTemperature())
        self.assertIsNone(sdk.getDriverTemperature())

    def test_future_product_sixty_four_axes_fragmentation_and_reconnect(self):
        self.configure_peer(255, 'future-product')
        config = profile(64, product_model_type=0xF0010001, revision=17)
        joint_id, _, name = config['joints'][-1]
        config['joints'][-1] = (0xFFFFFFFF, 255, name)
        self.configure_catalog(config)
        sdk, port = self.connect(mirrorme_robot_sdk.RequestRobotState)
        self.assertEqual(sdk.getRobotModel(), mirrorme_robot_sdk.RobotModel.Unknown)
        self.assertEqual(self.query_peer('capability_requests'), [(1, []), (2, [(1, 1, 0)])])
        state = self.session(port)
        records = self.records(config)
        packets = temperatures(state['token'], 17, records, fragment_size=32)
        self.send(port, packets[1:])
        time.sleep(.03)
        self.assertIsNone(sdk.getJointTemperatures(), 'incomplete snapshots must not leak partial arrays')
        self.send(port, packets[:1])
        self.wait(lambda: sdk.getMotorTemperature() is not None)
        self.assertEqual(sdk.getMotorTemperature(), [record[1] for record in records])
        self.assertEqual(len(sdk.getDriverTemperature()), 64)
        self.assertEqual(len(sdk.getJointTemperatures()['joints']), 64)
        self.assertEqual([joint['joint_id'] for joint in sdk.getJointTemperatures()['joints']],
                         [joint[0] for joint in config['joints']])

        sdk.disconnect()
        self.assertIsNone(sdk.getJointTemperatures())
        self.assertTrue(sdk.connect())
        self.wait(sdk.isConnected)
        replacement = self.session(port)
        self.assertNotEqual(replacement['token'], state['token'])
        self.send(port, temperatures(state['token'], 17, records, sequence=99))
        time.sleep(.03)
        self.assertIsNone(sdk.getJointTemperatures(), 'previous session token must be rejected')
        self.send(port, temperatures(replacement['token'], 17, records))
        self.wait(lambda: sdk.getJointTemperatures() is not None)
        self.assertEqual(sdk.getJointTemperatures()['sequence'], 1)

    def test_unsupported_temperature_version_is_reported_without_activation(self):
        self.configure_peer(3, 'future-temperature-version')
        config = profile()
        config['supported'] = [(1, 2, 0), (0x90010001, 9, 0)]
        self.configure_catalog(config)
        sdk, port = self.connect(mirrorme_robot_sdk.RequestRobotState)
        for _, selected in self.query_peer('capability_requests'):
            self.assertEqual(selected, [], 'unknown feature versions must not be selected')
        session = max((item for item in self.query_peer('capability_sessions') if item['port'] == port),
                      key=lambda item: item['token'])
        self.send(port, temperatures(session['token'], 7, self.records(config)))
        time.sleep(.03)
        self.assertIsNone(sdk.getJointTemperatures())
        self.assertIsNone(sdk.getMotorTemperature())

    def test_legacy_fallback_is_limited_to_identified_bpx_products(self):
        for wire_model in (0, 1, 2, 3, 255):
            with self.subTest(model=wire_model):
                self.configure_peer(wire_model)
                self.query_peer('clear_requests')
                sdk, port = self.connect(mirrorme_robot_sdk.RequestRobotState)
                self.assertFalse(any(request[10] == 7 for request in self.query_peer('requests')))
                body = struct.pack('<BB2xf24b', 75, 0, 0.0, *([5] * 12), *([7] * 12))
                self.send(port, [struct.pack('<HHI', 1, len(body), 1234) + body])
                self.wait(lambda: sdk.getBatteryLevel() == 75)
                if wire_model in (0, 1):
                    self.wait(lambda: sdk.getJointTemperatures() is not None)
                    snapshot = sdk.getJointTemperatures()
                    self.assertEqual(snapshot['topology_revision'], 0)
                    self.assertEqual(snapshot['timestamp_ms'], 1234)
                    self.assertEqual(len(snapshot['joints']), 12)
                    self.assertEqual(sdk.getMotorTemperature(), [65.0] * 12)
                    self.assertEqual(sdk.getDriverTemperature(), [67.0] * 12)
                else:
                    self.assertIsNone(sdk.getJointTemperatures())
                    self.assertIsNone(sdk.getMotorTemperature())
                    self.assertIsNone(sdk.getDriverTemperature())
                sdk.disconnect()


if __name__ == '__main__':
    unittest.main()

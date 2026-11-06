"""Independent little-endian capability fixtures for hardware-free SDK tests."""
import struct

DISCOVERY_TAG = 0x31504143


def tlv(kind, body, flags=0):
    return struct.pack('<HHI', kind, flags, len(body)) + body


def catalog(config, enabled=()):
    joints = config['joints']
    body = tlv(1, struct.pack('<III', config['product_model_type'],
                              config['topology_revision'], len(joints)))
    for feature in config.get('supported', [(1, 1, 0)]):
        body += tlv(2, struct.pack('<IHH', *feature))
    for joint_id, joint_type, name in joints:
        name = name.encode('utf-8')
        body += tlv(3, struct.pack('<IBBH', joint_id, joint_type, 0, len(name)) + name)
    for feature in enabled:
        body += tlv(4, struct.pack('<IHH', *feature))
    if config.get('unknown_tlv'):
        body += tlv(0x7F01, b'future optional parameters', 0x8000)
    return body


def response(session, operation, request_id, token, body):
    # Exactly 32 bytes before the explicitly sized TLV body.
    return struct.pack('<HBBHBBIIIQI', session, 1, operation, 1, 7, 0,
                       request_id, 0, len(body), token, DISCOVERY_TAG) + body


def selection_features(body):
    features = []
    while body:
        kind, flags, length = struct.unpack_from('<HHI', body)
        value, body = body[8:8 + length], body[8 + length:]
        if kind == 4:
            features.append(struct.unpack('<IHH', value))
    return features


def temperatures(token, revision, records, sequence=1, timestamp=None,
                 fragment_size=72):
    """records are (joint_id, optional motor Celsius, optional driver Celsius)."""
    if timestamp is None:
        timestamp = 1234567890123 + sequence - 1
    packets = []
    for start in range(0, len(records), fragment_size):
        part = records[start:start + fragment_size]
        payload = struct.pack('<HHQIIQHHHH', 1, 0, token, sequence, revision,
                              timestamp, len(records), start, len(part), 0)
        for joint_id, motor, driver in part:
            valid = (motor is not None) | ((driver is not None) << 1)
            payload += struct.pack('<IHHff', joint_id, valid, 0,
                                   motor if motor is not None else 0,
                                   driver if driver is not None else 0)
        packets.append(struct.pack('<HHI', 0x00E1, len(payload), timestamp & 0xFFFFFFFF) + payload)
    return packets


def profile(count=16, product_model_type=3, revision=7):
    # Deliberately non-contiguous IDs: array indexes are not joint IDs.
    return {'product_model_type': product_model_type, 'topology_revision': revision,
            'supported': [(1, 1, 0), (0x90010001, 9, 0x4000)], 'unknown_tlv': True,
            'joints': [(100 + 3 * i, 2 if i % 4 == 3 else 1, f'joint_{i}')
                       for i in range(count)]}


def joint_states(token, revision, records, sequence=1, timestamp=None,
                 fragment_size=57):
    """Independent E2 frames: (joint_id, optional position, velocity, torque)."""
    if timestamp is None:
        timestamp = 1234567890123 + sequence - 1
    packets = []
    for start in range(0, len(records), fragment_size):
        part = records[start:start + fragment_size]
        payload = struct.pack('<HHQIIQHHHH', 1, 0, token, sequence, revision,
                              timestamp, len(records), start, len(part), 0)
        for joint_id, position, velocity, torque in part:
            values = (position, velocity, torque)
            valid = sum((value is not None) << bit for bit, value in enumerate(values))
            payload += struct.pack('<IHHfff', joint_id, valid, 0,
                                   *(value if value is not None else 0 for value in values))
        packets.append(struct.pack('<HHI', 0x00E2, len(payload), timestamp & 0xFFFFFFFF) + payload)
    return packets


def joint_profile(count=16, product_model_type=3, revision=7):
    config = profile(count, product_model_type, revision)
    config['supported'].insert(1, (2, 1, 0))
    return config

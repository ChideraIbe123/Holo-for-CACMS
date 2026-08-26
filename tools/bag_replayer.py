#!/usr/bin/env python3
"""Replay the lab's real BlueROV2 mcap rosbag onto live ROS 2 topics with original
timing — no rosbag2/mcap storage plugin needed (pure-python mcap reader).

Republishes exactly the topics the dead-reckon pipeline consumes, plus the DVL
vendor's onboard dead-reckoning pose as the comparison reference:
  /mavros/imu/data, /dvl/twist, /mavros/global_position/rel_alt,
  /dvl/fom, /dvl/velocity_valid, /dvl/dead_reckoning/pose

Usage: python3 bag_replayer.py <path-to.mcap> [rate]
"""
import sys
import time

import rclpy
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Float64, Float32, Bool


def _hdr(dst, src):
    dst.header.stamp.sec = src.header.stamp.sec
    dst.header.stamp.nanosec = src.header.stamp.nanosec
    dst.header.frame_id = src.header.frame_id


def conv_imu(m):
    out = Imu()
    _hdr(out, m)
    for f in ('x', 'y', 'z', 'w'):
        setattr(out.orientation, f, float(getattr(m.orientation, f)))
    for f in ('x', 'y', 'z'):
        setattr(out.angular_velocity, f, float(getattr(m.angular_velocity, f)))
        setattr(out.linear_acceleration, f, float(getattr(m.linear_acceleration, f)))
    out.orientation_covariance = [float(v) for v in m.orientation_covariance]
    out.angular_velocity_covariance = [float(v) for v in m.angular_velocity_covariance]
    out.linear_acceleration_covariance = [float(v) for v in m.linear_acceleration_covariance]
    return out


def conv_twist(m):
    out = TwistStamped()
    _hdr(out, m)
    for f in ('x', 'y', 'z'):
        setattr(out.twist.linear, f, float(getattr(m.twist.linear, f)))
        setattr(out.twist.angular, f, float(getattr(m.twist.angular, f)))
    return out


def conv_pose(m):
    out = PoseStamped()
    _hdr(out, m)
    for f in ('x', 'y', 'z'):
        setattr(out.pose.position, f, float(getattr(m.pose.position, f)))
    for f in ('x', 'y', 'z', 'w'):
        setattr(out.pose.orientation, f, float(getattr(m.pose.orientation, f)))
    return out


def conv_f64(m):
    out = Float64()
    out.data = float(m.data)
    return out


def conv_f32(m):
    out = Float32()
    out.data = float(m.data)
    return out


def conv_bool(m):
    out = Bool()
    out.data = bool(m.data)
    return out


TOPICS = {
    '/mavros/imu/data': (Imu, conv_imu),
    '/dvl/twist': (TwistStamped, conv_twist),
    '/mavros/global_position/rel_alt': (Float64, conv_f64),
    '/dvl/fom': (Float32, conv_f32),
    '/dvl/velocity_valid': (Bool, conv_bool),
    '/dvl/dead_reckoning/pose': (PoseStamped, conv_pose),
}


def main():
    bag = sys.argv[1]
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

    rclpy.init()
    node = rclpy.create_node('bag_replayer')
    pubs = {t: node.create_publisher(mt, t, 50) for t, (mt, _) in TOPICS.items()}

    with open(bag, 'rb') as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        msgs = [(msg.log_time, ch.topic, decoded)
                for schema, ch, msg, decoded in reader.iter_decoded_messages(topics=list(TOPICS))]
    msgs.sort(key=lambda x: x[0])
    node.get_logger().info(f'replaying {len(msgs)} messages over '
                           f'{(msgs[-1][0] - msgs[0][0]) / 1e9:.1f} s (rate {rate}x)')

    t0_bag = msgs[0][0]
    t0_wall = time.time()
    counts = {t: 0 for t in TOPICS}
    for log_time, topic, decoded in msgs:
        target = (log_time - t0_bag) / 1e9 / rate
        lag = target - (time.time() - t0_wall)
        if lag > 0:
            time.sleep(lag)
        pubs[topic].publish(TOPICS[topic][1](decoded))
        counts[topic] += 1

    node.get_logger().info(f'done: {counts}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

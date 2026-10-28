#!/usr/bin/env python3
"""Converte la registrazione di cuVSLAM in un MCAP che Rerun visualizza in 3D.

Rerun legge da MCAP PointCloud2, PoseStamped e TFMessage, ma non Odometry e Path.
Questo script copia i topic già leggibili e converte gli altri:

  /tf, /tf_static                      copiati (posa della camera nel tempo)
  PointCloud2 (es. landmark cuVSLAM)   copiati
  /visual_slam/tracking/odometry  ->   /rerun/odometry/pose   PoseStamped (posa corrente)
                                  ->   /rerun/odometry/trail  PointCloud2 (traiettoria fino a quell'istante)
  nav_msgs/Path (vo_path, slam_path) ->  /rerun/<nome>          PointCloud2, un punto per posa
  geometry_msgs/PoseArray (pose graph) -> /rerun/<nome>         PointCloud2

Gira nel container ROS 2 (usa rosbag2_py, rclpy, sensor_msgs_py), senza pip.
Uso: python3 to_rerun_mcap.py <cartella bag di input> <cartella bag di output>
"""
import os
import sys

import rosbag2_py
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32

COPY_TYPES = {
    'tf2_msgs/msg/TFMessage',
    'sensor_msgs/msg/PointCloud2',
    'geometry_msgs/msg/PoseStamped',
}
ODOM_TYPE = 'nav_msgs/msg/Odometry'
PATH_TYPE = 'nav_msgs/msg/Path'
POSE_ARRAY_TYPE = 'geometry_msgs/msg/PoseArray'
CLOUD_TYPE = 'sensor_msgs/msg/PointCloud2'


def rerun_name(topic):
    # /visual_slam/tracking/slam_path -> /rerun/slam_path
    return '/rerun/' + topic.rstrip('/').split('/')[-1]


def topic_metadata(i, name, type_):
    try:
        return rosbag2_py.TopicMetadata(id=i, name=name, type=type_, serialization_format='cdr')
    except TypeError:  # versioni di rosbag2 senza il campo id
        return rosbag2_py.TopicMetadata(name=name, type=type_, serialization_format='cdr')


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]
    if os.path.exists(dst):
        sys.exit(f'{dst} esiste già')

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=src, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=dst, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))

    created = {}

    def out_topic(name, type_):
        if name not in created:
            writer.create_topic(topic_metadata(len(created), name, type_))
            created[name] = 0
        created[name] += 1
        return name

    msg_class = {}

    def decode(topic, data):
        if topic not in msg_class:
            msg_class[topic] = get_message(types[topic])
        return deserialize_message(data, msg_class[topic])

    trail = []
    skipped = set()
    while reader.has_next():
        topic, data, t = reader.read_next()
        type_ = types[topic]

        if type_ in COPY_TYPES:
            writer.write(out_topic(topic, type_), data, t)

        elif type_ == ODOM_TYPE:
            odom = decode(topic, data)
            pose = PoseStamped()
            pose.header = odom.header
            pose.pose = odom.pose.pose
            writer.write(out_topic('/rerun/odometry/pose', 'geometry_msgs/msg/PoseStamped'),
                         serialize_message(pose), t)
            p = odom.pose.pose.position
            trail.append((p.x, p.y, p.z))
            cloud = create_cloud_xyz32(odom.header, trail)
            writer.write(out_topic('/rerun/odometry/trail', CLOUD_TYPE), serialize_message(cloud), t)

        elif type_ in (PATH_TYPE, POSE_ARRAY_TYPE):
            msg = decode(topic, data)
            poses = [ps.pose for ps in msg.poses] if type_ == PATH_TYPE else list(msg.poses)
            pts = [(q.position.x, q.position.y, q.position.z) for q in poses]
            cloud = create_cloud_xyz32(msg.header, pts)
            writer.write(out_topic(rerun_name(topic), CLOUD_TYPE), serialize_message(cloud), t)

        else:
            skipped.add(f'{topic} ({type_})')

    del writer  # chiude il file e scrive metadata.yaml

    print(f'Scritto {dst}')
    for name, n in sorted(created.items()):
        print(f'  {name}: {n}')
    if skipped:
        print('Non convertiti (Rerun non li visualizza):')
        for s in sorted(skipped):
            print(f'  {s}')


if __name__ == '__main__':
    main()
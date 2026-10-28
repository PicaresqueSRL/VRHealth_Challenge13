#!/usr/bin/env python3
"""Estrae in CSV le traiettorie dalle registrazioni di cuVSLAM fatte con slam_cam2.sh.

Input:   <slam_runs>/<config>/<bag>/bag/          (registrazione originale)
Output:  <csv_root>/<config>/<bag>/
           odometry.csv     una posa per messaggio di /visual_slam/tracking/odometry
           vo_path.csv      ultimo /visual_slam/tracking/vo_path
           slam_poses.csv   ultimo /visual_slam/tracking/slam_path = traiettoria SLAM finale,
                            ottimizzata con i loop closure
           status.csv       stato del tracking per frame (/visual_slam/status)

Colonne delle pose (come nei CSV della repo):
  timestamp,x,y,z,qx,qy,qz,qw,roll_deg,pitch_deg,yaw_deg
Pose in metri, relative alla posa iniziale della camera.

Gira nel container ROS 2 (usa rosbag2_py), senza pip.
Uso:
  python3 extract_paths.py [slam_runs] [csv_root]        tutti i bag
                          (default: /workspaces/isaac_ros-dev/slam_runs e .../slam_csv)
  python3 extract_paths.py --bag <cartella bag> <out>     un solo bag
"""
import csv
import glob
import math
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

WS = '/workspaces/isaac_ros-dev'
ODOM = '/visual_slam/tracking/odometry'
VO_PATH = '/visual_slam/tracking/vo_path'
SLAM_PATH = '/visual_slam/tracking/slam_path'
STATUS = '/visual_slam/status'
POSE_HEADER = ['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
               'roll_deg', 'pitch_deg', 'yaw_deg']


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def rpy_deg(qx, qy, qz, qw):
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return [math.degrees(a) for a in (roll, pitch, yaw)]


def pose_row(stamp, pose):
    p, q = pose.position, pose.orientation
    return [stamp_s(stamp), p.x, p.y, p.z, q.x, q.y, q.z, q.w, *rpy_deg(q.x, q.y, q.z, q.w)]


def write_csv(path, header, rows):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def extract(bag_dir, out_dir):
    if not os.path.isfile(os.path.join(bag_dir, 'metadata.yaml')):
        raise FileNotFoundError(f'{os.path.abspath(bag_dir)} non è una registrazione chiusa (manca metadata.yaml)')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = [t for t in (ODOM, VO_PATH, SLAM_PATH, STATUS) if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))

    classes = {}
    for t in wanted:
        try:
            classes[t] = get_message(types[t])
        except Exception:
            print(f'  tipo {types[t]} non disponibile, salto {t}')

    odom, status, last_path = [], [], {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in classes:
            continue
        if topic in (VO_PATH, SLAM_PATH):
            # i Path crescono a ogni frame: si decodifica solo l'ultimo
            last_path[topic] = data
            continue
        msg = deserialize_message(data, classes[topic])
        if topic == ODOM:
            odom.append(pose_row(msg.header.stamp, msg.pose.pose))
        else:
            status.append([stamp_s(msg.header.stamp), msg.vo_state,
                           getattr(msg, 'node_callback_execution_time', '')])

    os.makedirs(out_dir, exist_ok=True)
    summary = []
    write_csv(os.path.join(out_dir, 'odometry.csv'), POSE_HEADER, odom)
    summary.append(f'odometry {len(odom)}')
    for topic, name in ((VO_PATH, 'vo_path.csv'), (SLAM_PATH, 'slam_poses.csv')):
        if topic in last_path:
            msg = deserialize_message(last_path[topic], classes[topic])
            rows = [pose_row(ps.header.stamp, ps.pose) for ps in msg.poses]
            write_csv(os.path.join(out_dir, name), POSE_HEADER, rows)
            summary.append(f'{name[:-4]} {len(rows)}')
        else:
            summary.append(f'{name[:-4]} assente')
    if status:
        write_csv(os.path.join(out_dir, 'status.csv'),
                  ['timestamp', 'vo_state', 'node_callback_execution_time'], status)
        summary.append(f'status {len(status)}')
    return ', '.join(summary)


def main():
    args = sys.argv[1:]
    if args[:1] == ['--bag']:
        if len(args) != 3:
            sys.exit(__doc__)
        try:
            print(extract(args[1], args[2]))
        except FileNotFoundError as e:
            sys.exit(f'ERRORE: {e}')
        return
    if len(args) > 2 or args[:1] in (['-h'], ['--help']):
        sys.exit(__doc__)

    runs = args[0] if len(args) > 0 else os.path.join(WS, 'slam_runs')
    out = args[1] if len(args) > 1 else os.path.join(WS, 'slam_csv')
    bags = sorted(glob.glob(os.path.join(runs, '*', '*', 'bag', 'metadata.yaml')))
    if not bags:
        sys.exit(f'Nessuna registrazione trovata in {runs}/<config>/<bag>/bag/')
    for meta in bags:
        bag_dir = os.path.dirname(meta)
        run_dir = os.path.dirname(bag_dir)
        config, bag = os.path.basename(os.path.dirname(run_dir)), os.path.basename(run_dir)
        try:
            print(f'[{config}/{bag}] {extract(bag_dir, os.path.join(out, config, bag))}')
        except Exception as e:  # un bag rotto non blocca gli altri
            print(f'[{config}/{bag}] ERRORE: {e}')


if __name__ == '__main__':
    main()
#!/usr/bin/env python2

# ****************************************************************************
# Copyright 2019 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************
# -*- coding: utf-8 -*-

# TODO this script could be implemented in a more generic way to get all fields
# from a struct and parse arbitray messages. for the moment it fits my needs
# adapted from record_channel_info

import sys
import csv

from cyber_py import cyber
from cyber_py import record
from cyber.proto import record_pb2
from modules.localization.proto import localization_pb2, imu_pb2, gps_pb2
from modules.planning.proto import planning_pb2


def print_channel_info(file_path):
    freader = record.RecordReader(file_path)
    channels = freader.get_channellist()

    header_msg = freader.get_headerstring()
    header = record_pb2.Header()
    header.ParseFromString(header_msg)

    count = 0
    print("######## Available channels: ########")
    for channel in channels:
        desc = freader.get_protodesc(channel)
        count += 1
        print(
            'Channel: %s, count: %d, desc size: %d' %
            (channel, count, len(desc)))

    rows_imu = []
    rows_odometry = []
    rows_loca = []
    rows_planning = []
    for channel_name, msg, datatype, timestamp in freader.read_messages():
        if channel_name == "/apollo/localization/pose":
            localization = localization_pb2.LocalizationEstimate()
            localization.ParseFromString(msg)
            t = localization.header.timestamp_sec
            x = localization.pose.position.x
            y = localization.pose.position.y
            z = localization.pose.position.z
            vx = localization.pose.linear_velocity.x
            vy = localization.pose.linear_velocity.y
            vz = localization.pose.linear_velocity.z
            ax = localization.pose.linear_acceleration.x
            ay = localization.pose.linear_acceleration.y
            az = localization.pose.linear_acceleration.z
            qx = localization.pose.orientation.qx
            qy = localization.pose.orientation.qy
            qz = localization.pose.orientation.qz
            qw = localization.pose.orientation.qw
            h = localization.pose.heading
            vrotx = localization.pose.angular_velocity.x
            vroty = localization.pose.angular_velocity.y
            vrotz = localization.pose.angular_velocity.z
            row = [t, x, y, z, vx, vy, vz, ax, ay, az,
                   qx, qy, qz, qw, h, vrotx, vroty, vrotz]
            rows_loca.append(row)

        if channel_name == "/apollo/sensor/gnss/corrected_imu":
            corrected_imu = imu_pb2.CorrectedImu()
            corrected_imu.ParseFromString(msg)
            t = corrected_imu.header.timestamp_sec
            ax = corrected_imu.imu.linear_acceleration.x
            ay = corrected_imu.imu.linear_acceleration.y
            az = corrected_imu.imu.linear_acceleration.z
            vrotx = corrected_imu.imu.angular_velocity.x
            vroty = corrected_imu.imu.angular_velocity.y
            vrotz = corrected_imu.imu.angular_velocity.z
            roll = corrected_imu.imu.euler_angles.x
            pitch = corrected_imu.imu.euler_angles.y
            yaw = corrected_imu.imu.euler_angles.z
            row = [t, ax, ay, az, vrotx, vroty, vrotz, roll, pitch, yaw]
            rows_imu.append(row)

        if channel_name == "/apollo/sensor/gnss/odometry":
            odometry = gps_pb2.Gps()
            odometry.ParseFromString(msg)
            t = odometry.header.timestamp_sec
            x = odometry.localization.position.x
            y = odometry.localization.position.y
            z = odometry.localization.position.z
            qx = odometry.localization.orientation.qx
            qy = odometry.localization.orientation.qy
            qz = odometry.localization.orientation.qz
            qw = odometry.localization.orientation.qw
            vx = odometry.localization.linear_velocity.x
            vy = odometry.localization.linear_velocity.y
            vz = odometry.localization.linear_velocity.z
            row = [t, x, y, z, qx, qy, qz, qw, vx, vy, vz]
            rows_odometry.append(row)

        if channel_name == "/apollo/planning":
            traj = planning_pb2.ADCTrajectory()
            traj.ParseFromString(msg)
            timestamp_sec = traj.header.timestamp_sec
            sequence_num = traj.header.sequence_num
            replan_reason = traj.replan_reason
            num_points = len(traj.trajectory_point)

            for traj_pt in traj.trajectory_point:
                relative_time = traj_pt.relative_time
                x = traj_pt.path_point.x
                y = traj_pt.path_point.y
                theta = traj_pt.path_point.theta
                kappa = traj_pt.path_point.kappa
                dkappa = traj_pt.path_point.dkappa
                s = traj_pt.path_point.s
                v = traj_pt.v
                a = traj_pt.a
                da = traj_pt.da
                steer = traj_pt.steer
                row = [timestamp_sec, relative_time, sequence_num, num_points, x, y,
                       theta, kappa, dkappa, s, v, a, da, steer, replan_reason]
                rows_planning.append(row)

    with open("imu.csv", "wt") as fp:
        writer = csv.writer(fp, delimiter=",")
        writer.writerow(["t", "ax", "ay", "az", "vrotx",
                         "vroty", "vrotz", "roll", "pitch", "yaw"])
        writer.writerows(rows_imu)

    with open("odometry.csv", "wt") as fp:
        writer = csv.writer(fp, delimiter=",")
        writer.writerow(["t", "x", "y", "z", "qx", "qy",
                         "qz", "qw", "vx", "vy", "vz"])
        writer.writerows(rows_odometry)

    with open("localization.csv", "wt") as fp:
        writer = csv.writer(fp, delimiter=",")
        writer.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "ax", "ay",
                         "az", "qx", "qy", "qz", "qw", "h", "vrotx", "vroty", "vrotz"])
        writer.writerows(rows_loca)

    with open("planning.csv", "wt") as fp:
        writer = csv.writer(fp, delimiter=",")
        writer.writerow(["timestamp_sec", "relative_time", "sequence_num", "num_points", "x", "y",
                         "theta", "kappa", "dkappa", "s", "v", "a", "da", "steer", "replan_reason"])
        writer.writerows(rows_planning)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: %s record_file' % sys.argv[0])
        sys.exit(0)

    cyber.init()
    print_channel_info(sys.argv[1])
    cyber.shutdown()

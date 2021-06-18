#!/usr/bin/python

# MIT License
#
# Copyright (c) 2021 Rik Baehnemann, ASL, ETH Zurich, Switzerland
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rosbag
from geometry_msgs.msg import PointStamped
import yaml
import pymap3d as pm
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import rospy

from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

def openCoordinates(file):
    with open(file, 'r') as stream:
        try:
            dict = yaml.safe_load(stream)
            lat = dict['lat_wgs84']
            lon = dict['lon_wgs84']
            alt = dict['alt_wgs84']
            return lat, lon, alt
        except yaml.YAMLError as exc:
            print(exc)

def alignDjiFrame(t_rtk, rtk, t_dji, dji, t_att, att, N=-1, method='mean', plot=False):
    print('Calibrating the DJI GPS data with the first {:d} (-1 is all) RTK messages.'.format(N))
    # Create interpolated data frame with RTK time stamps.
    # Correct DJI altitude by rtk take off altitude. DJI altitude is extremly off.
    dji[:,2] += rtk[0,2] - dji[0,2]

    # Interpolate position.
    df_rtk = pd.DataFrame(data=rtk, index=pd.Index(t_rtk, dtype='datetime64[ns]'), columns=['x_rtk', 'y_rtk', 'z_rtk'])
    df_dji = pd.DataFrame(data=dji, index=pd.Index(t_dji, dtype='datetime64[ns]'), columns=['x_dji', 'y_dji', 'z_dji'])
    df = df_rtk.merge(df_dji, left_index=True, right_index=True, how='outer')
    df[df_dji.columns] = df[df_dji.columns].interpolate(method='index')
    df.dropna(inplace=True)

    # Interpolate quaternion.
    key_rots = R.from_quat(att)
    key_times = t_att/1.0e9
    t_reduced = t_rtk
    if N > 0:
        t_reduced = t_reduced[0:N]
    times = t_reduced/1.0e9
    slerp = Slerp(key_times, key_rots)
    interp_rots = slerp(times)

    df_att = pd.DataFrame(data=interp_rots.as_quat(), index=pd.Index(t_reduced, dtype='datetime64[ns]'), columns=['qx', 'qy', 'qz', 'qw'])
    df = df.merge(df_att, left_index=True, right_index=True, how='outer')
    df.dropna(inplace=True)

    # For every time stamp compute calibration.
    # DJI_r_DJI_RTK = R_ENU_DJI^-1 * (WORLD_r_WORLD_RTK - WORLD_r_WORLD_DJI)
    calibration = np.zeros([len(df.index), 3])
    WORLD_r_WORLD_RTK = df[df_rtk.columns].to_numpy()
    WORLD_r_WORLD_DJI = df[df_dji.columns].to_numpy()
    R_ENU_DJI = R.from_quat(df[df_att.columns].to_numpy())
    DJI_r_DJI_RTK = R_ENU_DJI.inv().apply(WORLD_r_WORLD_RTK - WORLD_r_WORLD_DJI)

    mean = np.mean(DJI_r_DJI_RTK, axis=0)
    median = np.median(DJI_r_DJI_RTK, axis=0)

    if plot:
        plotCalibration(DJI_r_DJI_RTK, mean, median, 0, 1) # x-y-axis
        plotCalibration(DJI_r_DJI_RTK, mean, median, 0, 2) # x-z-axis
        plotCalibration(DJI_r_DJI_RTK, mean, median, 1, 2) # y-z-axis

    print('Mean DJI_r_DJI_RTK: [{:.3f}, {:.3f}, {:.3f}]'.format(mean[0], mean[1], mean[2]))
    print('Median DJI_r_DJI_RTK: [{:.3f}, {:.3f}, {:.3f}]'.format(median[0], median[1], median[2]))
    print('First DJI_r_DJI_RTK: [{:.3f}, {:.3f}, {:.3f}]'.format(DJI_r_DJI_RTK[0,0], DJI_r_DJI_RTK[0,1], DJI_r_DJI_RTK[0,2]))

    # Convert DJI positions into RTK frame.
    correction = np.zeros(3)
    if method=='mean':
        print('Correcting with mean of calibration.')
        correction=mean
    elif method=='median':
        print('Correcting with median of calibration.')
        correction=DJI_r_DJI_RTK[0,:]
    elif method=='first':
        print('Correcting with RTK message.')
        correction=median
    dji_times = t_dji/1.0e9
    dji_interp_rots = slerp(dji_times)
    dji_corrected = dji + dji_interp_rots.apply(correction)

    if plot:
        df_dji_corrected = pd.DataFrame(data=dji_corrected, index=pd.Index(t_dji, dtype='datetime64[ns]'), columns=['x_dji', 'y_dji', 'z_dji'])
        ax = df_dji_corrected.plot()
        df_rtk.plot(ax=ax, title='Trajectory after calibration: {:s} Samples: {:d}'.format(method, N))
        plt.show()

    return dji_corrected

def plotCalibration(DJI_r_DJI_RTK, mean, median, x, y):
    plt.scatter(x=DJI_r_DJI_RTK[:,x], y=DJI_r_DJI_RTK[:,y])
    plt.scatter(x=mean[x], y=mean[y], label='mean')
    plt.scatter(x=median[x], y=median[y], label='median')
    plt.scatter(x=DJI_r_DJI_RTK[0,x], y=DJI_r_DJI_RTK[0,y], label='first')
    plt.scatter(x=DJI_r_DJI_RTK[-1,x], y=DJI_r_DJI_RTK[-1,y], label='last')
    plt.grid()
    plt.legend()
    plt.title('DJI_r_DJI_RTK')
    if x == 0:
        plt.xlabel('x [m]')
    elif x == 1:
        plt.xlabel('y [m]')
    elif x == 2:
        plt.xlabel('z [m]')
    if y == 0:
        plt.ylabel('x [m]')
    elif y == 1:
        plt.ylabel('y [m]')
    elif y == 2:
        plt.ylabel('z [m]')
    plt.show()


def getStamp(msg):
    return msg.header.stamp.secs * 1e9 + msg.header.stamp.nsecs

def convert_dji_positions(input_bag, output_bag, rtk_topic, navsat_topic, attitude_topic, enu_origin, plot):
    enu_lat, enu_lon, enu_alt = openCoordinates(enu_origin)
    print('ENU origin WGS84: [{:.5f}, {:.5f}, {:.5f}]'.format(enu_lat, enu_lon, enu_alt))

    input = rosbag.Bag(input_bag, 'r')

    # Align DJI GPS to RTK GPS.
    # Read RTK messages, DJI attitude and DJI GPS position
    topics = [navsat_topic, rtk_topic, attitude_topic]
    info = input.get_type_and_topic_info(topics)[1]
    for topic in topics:
        if topic == navsat_topic:
            dji = np.zeros([info[topic].message_count, 3])
            t_dji = np.zeros([info[topic].message_count], dtype=np.uint64)
        elif topic == rtk_topic:
            rtk = np.zeros([info[topic].message_count, 3])
            t_rtk = np.zeros([info[topic].message_count], dtype=np.uint64)
        elif topic == attitude_topic:
            att = np.zeros([info[topic].message_count, 4])
            t_att = np.zeros([info[topic].message_count], dtype=np.uint64)

    dji_idx = 0
    rtk_idx = 0
    att_idx = 0
    for topic, msg, t in input.read_messages(topics=topics):
        if topic == navsat_topic:
            x, y, z = pm.geodetic2enu(msg.latitude, msg.longitude, msg.altitude, enu_lat, enu_lon, enu_alt)
            t_dji[dji_idx] = getStamp(msg)
            dji[dji_idx, :] = [x, y, z]
            dji_idx += 1
        elif topic == rtk_topic:
            t_rtk[rtk_idx] = getStamp(msg)
            rtk[rtk_idx, :] = [msg.point.x, msg.point.y, msg.point.z]
            rtk_idx += 1
        elif topic == attitude_topic:
            t_att[att_idx] = getStamp(msg)
            att[att_idx, :] = [msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w]
            att_idx += 1

    print('Loaded {:d} RTK position messages.'.format(len(rtk)))
    print('Loaded {:d} DJI position messages.'.format(len(dji)))
    print('Loaded {:d} DJI attitude position messages.'.format(len(att)))

    dji_aligned = alignDjiFrame(t_rtk, rtk, t_dji, dji, t_att, att, -1, method='only z', plot=plot)

    # Write output.
    output = rosbag.Bag(output_bag, 'w')
    for (t, msg) in zip(t_dji, dji_aligned):
         p = PointStamped()
         p.header.stamp = rospy.Time(0, t)
         p.header.frame_id = 'enu'
         p.point.x = msg[0]
         p.point.y = msg[1]
         p.point.z = msg[2]
         output.write(navsat_topic + '_enu', p, p.header.stamp)

    for topic, msg, t in input.read_messages(topics=[attitude_topic]):
        output.write(topic, msg, msg.header.stamp)

    input.close()
    output.close()

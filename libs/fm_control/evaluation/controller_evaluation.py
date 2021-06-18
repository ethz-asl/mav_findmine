#!/usr/bin/env python

# MIT License
#
# Copyright (c) 2020 Rik Baehnemann, ASL, ETH Zurich, Switzerland
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

import rosbag_pandas
import math
import numpy as np

def yawFromQuaternion(qx,qy,qz,qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

def quatToYaw(qx, qy, qz, qw):
    yaw = []
    for index, item in enumerate(qx):
        yaw.append(yawFromQuaternion(qx[index],qy[index],qz[index],qw[index]))
    return yaw

def calculateReferenceTrackingError(df, namespace):
    df_ref = df.copy()
    x_ref_topic = namespace + 'position_controller_pos_yaw_ref__pose_position_x'
    y_ref_topic = namespace + 'position_controller_pos_yaw_ref__pose_position_y'
    z_ref_topic = namespace + 'position_controller_pos_yaw_ref__pose_position_z'
    x_topic = namespace + 'dji_sdk_local_position__point_x'
    y_topic = namespace + 'dji_sdk_local_position__point_y'
    z_topic = namespace + 'dji_sdk_local_position__point_z'

    df_ref = df_ref.filter(items=[x_ref_topic, y_ref_topic, z_ref_topic, x_topic, y_topic, z_topic])

    x_ref = df_ref[x_ref_topic].values
    y_ref = df_ref[y_ref_topic].values
    z_ref = df_ref[z_ref_topic].values
    x = df_ref[x_topic].interpolate(method='time').values
    y = df_ref[y_topic].interpolate(method='time').values
    z = df_ref[z_topic].interpolate(method='time').values

    data = np.array([x_ref,y_ref,z_ref,x,y,z])
    data = data[:,~np.any(np.isnan(data), axis=0)]

    error = data[3:6,:] - data[0:3,:]
    error_norm = np.linalg.norm(error, axis=0)
    print("RMSE [m]: " + str(np.mean(error_norm)))

# rosbag filter 2018-12-03-17-46-44.bag 2018-12-03-17-46-44_controls.bag "topic == '/moa/position_controller/pos_yaw_ref' or topic == '/moa/dji_sdk/local_position' or topic == '/moa/dji_sdk/attitude'"


df = rosbag_pandas.bag_to_dataframe('control_2018-07-29-16-06-14.bag')

for c in df.columns:
    print c, df[c].min(),  df[c].max()

namespace='moa_'
calculateReferenceTrackingError(df, namespace)

qx = df[namespace + 'dji_sdk_attitude__quaternion_x'].values
qy = df[namespace + 'dji_sdk_attitude__quaternion_y'].values
qz = df[namespace + 'dji_sdk_attitude__quaternion_z'].values
qw = df[namespace + 'dji_sdk_attitude__quaternion_w'].values
x = df[namespace + 'dji_sdk_local_position__point_x'].interpolate().values
y = df[namespace + 'dji_sdk_local_position__point_y'].interpolate().values
z = df[namespace + 'dji_sdk_local_position__point_z'].interpolate().values

x_ref = df[namespace + 'position_controller_pos_yaw_ref__pose_position_x'].interpolate().values
y_ref = df[namespace + 'position_controller_pos_yaw_ref__pose_position_y'].interpolate().values
z_ref = df[namespace + 'position_controller_pos_yaw_ref__pose_position_z'].interpolate().values

yaw = quatToYaw(qx, qy, qz, qw)

# Template for matplotlibs with
# proper figsize, font, etc,

import matplotlib.pyplot as plt
import matplotlib

half_page_width = (5.0,5.0)
full_page_width = (6.5,2.25)
full_page_width_double = (6.5,4.5)

#########
# Select proper size of the plot here.
#	half_page_width: 	Regular plot (usually there are 2 of
#			 	those next to each other)
#
#	full_page_width: 	Plot that spans the full width of the
#				page (same size as 2 subplots)
#
#	full_page_width_double: Plot that spans full width of page and
#			        has double height (e.g. 4 subplots)
size =  half_page_width
#
#
# Define Title and activate/deactivate title plotting
title = "A plot $\\alpha \\beta \\gamma \\Phi \\Sigma$"
plot_title = False
#
#########

# set font
matplotlib.rcParams['font.family'] = "sans-serif"
matplotlib.rcParams['font.sans-serif'] = ["DejaVu Sans"]
matplotlib.rcParams.update({'font.size': 14})
matplotlib.rcParams.update({'legend.fontsize': 14})

# set linewidth for frames
matplotlib.rcParams['axes.linewidth'] = 2.0 #set the value globally

matplotlib.rcParams['xtick.labelsize'] = 14
matplotlib.rcParams['ytick.labelsize'] = 14

# create plot object
fig, ax = plt.subplots(figsize=size)
fig.patch.set_facecolor('white')


# plot stuff
data = np.array([x,y,yaw])
data = data[:,~np.any(np.isnan(data), axis=0)]
line_pos, = plt.plot(data[0], data[1], linewidth=2)
plt.quiver(data[0][::20], data[1][::20], np.cos(data[2])[::20], np.sin(data[2])[::20], linewidth=2)

# plot reference trajectory.
pos_ref = np.array([x_ref,y_ref])
line_ref, = plt.plot(pos_ref[0], pos_ref[1], color='blue', linewidth=2, linestyle='-')

ax.grid(color="k", alpha=0.1, linewidth=1, linestyle="-")

# label that stuff
if plot_title:
	plt.title(title)

plt.xlabel("East [m]")
plt.ylabel("North [m]")
plt.legend([line_pos, line_ref], ['DJI position', 'Control reference'])


# adjust margins
if plot_title:
	top_margin = 1-0.4/size[1]
else:
	top_margin = 1-0.2/size[1]

plt.subplots_adjust(left=0.7/size[0], right=1-0.15/size[0], top=top_margin, bottom=0.5/size[1])

# yay
plt.show()

fig.savefig("yaw.png",bbox_inches='tight',dpi=300, format='png')

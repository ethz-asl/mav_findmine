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
import rospy
import numpy as np
import tf
import tf2_ros
import geometry_msgs.msg
from datetime import datetime

def getStamp(stamp):
    dt = datetime.utcfromtimestamp(stamp.secs)
    us = stamp.nsecs // 1e3
    us_mod = stamp.nsecs % 1e3
    return "%d,%d,%d,%d,%d,%d,%d.%d" % (dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, us, us_mod)

def getTranslation(translation):
    return "%f,%f,%f" %(translation.x, translation.y, translation.z)

def getRotation(rotation):
    return "%f,%f,%f,%f" %(rotation.w, rotation.x, rotation.y, rotation.z)

def export_raw_positions(input_bag, out_folder, rtk_topic, attitude_topic):
    bag = rosbag.Bag(input_bag)

    br = tf2_ros.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    prev_att = None
    next_att = None
    rtk = None

    keys = ['a1', 'a2', 'a3', 'a4', 'FLU']
    f = {}
    for key in keys:
        csv_file = out_folder + '/' + key + '.csv'
        f[key] = open(csv_file, 'w')
        f[key].write('year, month, day, hour, min, second, microsecond, x, y, z, qw, qx, qy, qz\n')

    for topic, msg, t in bag.read_messages(topics=[rtk_topic, attitude_topic]):
        if topic == attitude_topic:
            prev_att = next_att
            next_att = msg
            if rtk:
                # Interpoplate attitude.
                q0 = [prev_att.quaternion.x, prev_att.quaternion.y, prev_att.quaternion.z, prev_att.quaternion.w]
                q1 = [next_att.quaternion.x, next_att.quaternion.y, next_att.quaternion.z, next_att.quaternion.w]
                dt = (next_att.header.stamp - prev_att.header.stamp).to_sec()
                s = (rtk.header.stamp - prev_att.header.stamp).to_sec()
                q = tf.transformations.quaternion_slerp(q0, q1, s / dt)

                # Publish transform
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rtk.header.stamp
                t.header.frame_id = 'enu'
                t.child_frame_id = 'rtk'
                t.transform.translation.x = rtk.point.x
                t.transform.translation.y = rtk.point.y
                t.transform.translation.z = rtk.point.z
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]

                br.sendTransform(t)
                rospy.sleep(0.01)

                # Lookup antenna transforms.
                try:
                    for key in f.keys():
                        T = tfBuffer.lookup_transform('enu', key, rtk.header.stamp)
                        f[key].write(getStamp(rtk.header.stamp))
                        f[key].write(",")
                        f[key].write(getTranslation(T.transform.translation))
                        f[key].write(",")
                        f[key].write(getRotation(T.transform.rotation))
                        f[key].write("\n")

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print("Failed to lookup TF.")

                rtk = None
        elif topic == rtk_topic:
            rtk = msg

    for key in keys:
        f[key].close()

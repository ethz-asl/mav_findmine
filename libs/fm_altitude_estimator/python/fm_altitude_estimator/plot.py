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
import matplotlib.pyplot as plt
import numpy as np

def plot(input_bag, state_topic, gnss_topic, radar_topic, lidar_topic):
    topics = [state_topic, gnss_topic, radar_topic, lidar_topic]
    bag = rosbag.Bag(input_bag)

    info = bag.get_type_and_topic_info(topics)[1]
    for topic in topics:
        if topic == state_topic:
            state = np.zeros([info[topic].message_count, 4])
        elif topic == gnss_topic:
            gnss = np.zeros([info[topic].message_count, 4])
        elif topic == radar_topic:
            radar = np.zeros([info[topic].message_count, 2])
        elif topic == lidar_topic:
            lidar = np.zeros([info[topic].message_count, 2])

    state_idx = 0
    gnss_idx = 0
    radar_idx = 0
    lidar_idx = 0

    for topic, msg, t in bag.read_messages(topics=topics):
        if topic == state_topic:
            state[state_idx, :] = [msg.header.stamp.to_sec(), msg.point.x, msg.point.y, msg.point.z]
            state_idx += 1
        elif topic == gnss_topic:
            gnss[gnss_idx, :] = [msg.header.stamp.to_sec(), msg.point.x, msg.point.y, msg.point.z]
            gnss_idx += 1
        elif topic == radar_topic:
            radar[radar_idx, :] = [msg.range.header.stamp.to_sec(), msg.range.range]
            radar_idx += 1
        elif topic == lidar_topic:
            lidar[lidar_idx, :] = [msg.range.header.stamp.to_sec(), msg.range.range]
            lidar_idx += 1

    print("Loaded %d altitude state messages." % (len(state)))
    print("Loaded %d gnss messages." % (len(gnss)))
    print("Loaded %d radar messages." % (len(radar)))
    print("Loaded %d lidar messages." % (len(lidar)))

    plt.plot(state[:,0], state[:,3], label='Est. altitude')
    plt.plot(gnss[:,0], gnss[:,3], label='DJI GNSS altitude')
    plt.plot(radar[:,0], radar[:,1], 'o', label='Radar range')
    plt.plot(lidar[:,0], lidar[:,1], 'o', label='Lidar range')
    plt.legend()
    plt.grid()
    plt.xlabel('Timestamp [s]')
    plt.ylabel('Altitude [m]')

    plt.show()

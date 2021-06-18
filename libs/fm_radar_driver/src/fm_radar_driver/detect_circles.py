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
from datetime import datetime

def detect_circles(input_bag, out_file, trajectory_topic):
    # Get trajectory start time and segment times from bag.
    bag = rosbag.Bag(input_bag)
    segment_times = []
    t = rospy.Time(0)
    for topic, msg, t in bag.read_messages(topics=[trajectory_topic]):
        t = msg.header.stamp
        end = t
        print('Start time: %d.%d' %(t.secs, t.nsecs))
        for segment in msg.segments:
            segment_times.append(segment.segment_time)
    # Search for consecutive identical segment times. These are the circle segments!
    diff = np.diff(segment_times)
    change = diff == rospy.Duration(0)
    start = []
    stop = []
    for idx in range(0, len(segment_times) - 2):
        t = t + segment_times[idx]
        if change[idx] == False and change[idx + 1] == True:
            start.append(t)
        elif change[idx] == True and change[idx + 1] == False:
            stop.append(t + segment_times[idx])
    # Merge
    segment_id = []
    for a, b in zip(start, stop):
        segment_id.append([a, b])
    print('Circle times:')
    print(segment_id)


    f = open(out_file, 'w')
    f.write('id,start,stop\n')
    id = 1
    for circle in segment_id:
        # Convert UTC to radar time (seconds since midnight)
        start_time = datetime.utcfromtimestamp(circle[0].to_sec())
        start_time = (start_time - start_time.replace(hour=0, minute=0, second=0, microsecond=0))
        end_time = datetime.utcfromtimestamp(circle[1].to_sec())
        end_time = (end_time - end_time.replace(hour=0, minute=0, second=0, microsecond=0))
        str = '{:02d},{:.6f},{:.6f}\n'.format(id, start_time.total_seconds(), end_time.total_seconds())
        f.write(str)
        id = id + 1

    f.close()

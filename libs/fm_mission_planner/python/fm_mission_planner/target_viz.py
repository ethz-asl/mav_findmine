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

import rospy
import rospkg
import pandas as pd
import pymap3d as pm
import os
import numpy as np
from matplotlib import cm
from matplotlib import colors

from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray

# Load target list from CSV, receive home point from ROS msgs and publish target points to RVIZ.
class TargetViz():
    def __init__(self):
        self.df_targets = None

        self.loadRosParameters()
        self.subscribeToTopics()
        self.advertiseTopics()

        self.loadTargetTable()

        self.main()

    def loadRosParameters(self):
        rospack = rospkg.RosPack()
        default_target_path = os.path.join(rospack.get_path('fm_mission_planner'), 'cfg/target_table.csv')
        self.target_path = rospy.get_param("~target_table", default_target_path)
        self.frame_id = rospy.get_param("~frame_id", 'enu')

    def subscribeToTopics(self):
        self.home_point_sub = rospy.Subscriber('home_point', NavSatFix, self.homePointCallback)

    def advertiseTopics(self):
        self.target_pub = rospy.Publisher('~targets', MarkerArray, latch=True)

    def homePointCallback(self, msg):
        self.lat0 = msg.latitude
        self.lon0 = msg.longitude
        self.alt0 = msg.altitude
        rospy.loginfo_throttle(10.0, 'Received home point lat0: ' + str(self.lat0) + ' lon0: ' + str(self.lon0) + ' alt0: ' + str(self.alt0))

        if self.df_targets is not None and len(self.df_targets):
            self.convertToENU()
            self.createColors()
            self.createMarkerArray()

            self.target_pub.publish(self.marker_array)

    def loadTargetTable(self):
        self.df_targets = pd.read_csv(self.target_path, sep=",")
        rospy.loginfo('Loading ' + str(len(self.df_targets)) + ' target points.')

    def convertToENU(self):
        lat = self.df_targets['lat'].values
        lon = self.df_targets['lon'].values
        alt = np.squeeze(np.zeros((len(self.df_targets), 1)))

        print lat
        print lat.size
        if lat.size == 1 and lon.size == 1 and alt.size == 1:
            lat = np.array([lat])
            lon = np.array([lon])
            alt = np.array([alt])

        self.east = []
        self.north = []
        self.up = []
        for i in range(0, len(self.df_targets)):
            east, north, up = pm.geodetic2enu(lat[i], lon[i], alt[i], self.lat0, self.lon0, self.alt0)
            self.east.append(east)
            self.north.append(north)
            self.up.append(up)

    def createColors(self):
        types = self.df_targets['type'].values
        color_map = cm.get_cmap('Set1')
        norm = colors.Normalize(vmin=min(types), vmax=max(types))
        self.colors = color_map(norm(types))

    def createMarkerArray(self):
        self.marker_array = MarkerArray()
        for i in range(0, len(self.df_targets)):
            marker = Marker()
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            marker.color.r = self.colors[i, 0]
            marker.color.g = self.colors[i, 1]
            marker.color.b = self.colors[i, 2]
            marker.color.a = self.colors[i, 3]
            marker.pose.position.x = self.east[i]
            marker.pose.position.y = self.north[i]
            marker.pose.position.z = self.up[i]
            marker.pose.orientation.w = 1.0
            marker.header.frame_id = self.frame_id
            marker.id = i

            self.marker_array.markers.append(marker)

    def main(self):
        rospy.spin()

#!/usr/bin/python

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
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from task_manager_lib.TaskClient import *

# The base mission class.
# Override getTaskParameters and runMission.
class Mission(object):
    def __init__(self):
        # Init ROS node.
        rospy.init_node("task_client")

    def main(self):
        self.getClientParameters()
        self.getTaskParameters()

        # Task manager client.
        self.tc = TaskClient(self.server_node, self.default_period)

        # Task execution.
        rospy.loginfo("Starting mission.")
        self.runMission()

    def getTaskParameters(self):
        rospy.logwarn("getTaskParameters not implemented.")

    def getClientParameters(self):
        self.server_node = rospy.get_param("~server","/moa/task_server")
        self.default_period = rospy.get_param("~default_period", 0.2)
        self.default_timeout_s = rospy.get_param("~default_timeout_s", -1.0)

    def runMission(self):
        rospy.logwarn("runMission not implemented.")

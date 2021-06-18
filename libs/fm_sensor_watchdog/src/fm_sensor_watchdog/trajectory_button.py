# MIT License
#
# Copyright (c) 2020 Lucas Streichenberg, ASL, ETH Zurich, Switzerland
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

class TrajectoryButton():
        def __init__(self, button):

            self.button = button
            self.button.clicked.connect(self.start_mission_service)
            self.button.setText("approve trajectory")
            self.service_name = rospy.get_param("~trajectory_service_name")

            self.service_available()

        def service_available(self): # check if service exists from time to time
            try:
                rospy.wait_for_service(self.service_name, 0.2)

                self.button.setStyleSheet("background-color: orange")
                return True

            except rospy.ROSException as exc:
                self.button.setStyleSheet("background-color: red")
                return False

        def start_mission_service(self):
            try:
                if self.service_available():
                    rospy.ServiceProxy(self.service_name, Empty)()
                    self.button.setStyleSheet("background-color: green")
                else:
                    rospy.logwarn("approve trajectory service not available!!")


            except rospy.ServiceException as exc:
                rospy.logwarn("approve trajectory service not available!!")
                self.button.setStyleSheet("background-color: red")

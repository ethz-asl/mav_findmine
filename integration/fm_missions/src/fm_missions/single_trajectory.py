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
import math
from fm_missions.mission import Mission
from task_manager_lib.TaskClient import *
from std_srvs.srv import SetBool

# For starting radar and sensor recording
import rospkg
import subprocess

class SingleTrajectory(Mission):
    def getTaskParameters(self):
        self.transition_flag_param = rospy.get_param("~mission_settings/transition", False)
        self.switch_control_frame_param = rospy.get_param("~mission_settings/switch_control_frame", False)
        self.record_radar_param = rospy.get_param("~mission_settings/record_radar", False)
        self.record_sensors_param = rospy.get_param("~mission_settings/record_sensors", False)
        self.record_cam_param = rospy.get_param("~mission_settings/record_cam", False)
        self.run_localization_optimization_param = rospy.get_param("~mission_settings/run_localization_optimization", False)

        # Take off.
        self.take_off_params = dict()
        self.take_off_params["timeout_s"] = rospy.get_param("~take_off/timeout_s", self.default_timeout_s)

        # Generate trajectory.
        self.generate_single_trajectory_params = dict()
        self.generate_single_trajectory_params["timeout_s"] = rospy.get_param("~generate_single_trajectory/timeout_s", self.default_timeout_s)
        self.generate_single_trajectory_params["task_period"] = rospy.get_param("~generate_single_trajectory/task_period", self.default_period)
        self.generate_single_trajectory_params["altitude"] = rospy.get_param("~generate_single_trajectory/altitude")
        self.generate_single_trajectory_params["transition_altitude"] = rospy.get_param("~generate_single_trajectory/transition_altitude")
        self.generate_single_trajectory_params["plan_from_home"] = not self.transition_flag_param

        rospy.loginfo("Getting mission file")
        self.generate_single_trajectory_params["mission_file"] = rospy.get_param("~generate_single_trajectory/mission_file")
        rospy.loginfo("Got mission file: %s" % self.generate_single_trajectory_params["mission_file"])

        # Execute trajectory.
        self.execute_single_trajectory_params = dict()
        self.execute_single_trajectory_params["timeout_s"] = rospy.get_param("~execute_single_trajectory/timeout_s", self.default_timeout_s)
        self.execute_single_trajectory_params["terminal_vel"] = rospy.get_param("~execute_single_trajectory/terminal_vel", 0.1)

        # Waypoint.
        self.waypoint_params = dict()
        self.waypoint_params["timeout_s"] = rospy.get_param("~waypoint/timeout_s", self.default_timeout_s)
        self.waypoint_params["ball_radius"] = rospy.get_param("~waypoint/ball_radius", 0.15)
        self.waypoint_params["terminal_vel"] = rospy.get_param("~waypoint/terminal_vel", 0.15)

        # Landing.
        self.landing_params = dict()
        self.landing_params["timeout_s"] = rospy.get_param("~landing/timeout_s", self.default_timeout_s)

        # Postprocessing.
        self.postprocessing_params = dict()
        self.postprocessing_params["timeout_s"] = rospy.get_param("~postprocessing/timeout_s", self.default_timeout_s)
        self.postprocessing_params["topics"] = rospy.get_param("~postprocessing/topics", "FLU")


    def runMission(self):
        # Generate single measurement trajectory.
        rospy.loginfo("Generating trajectory")
        try:
            self.tc.GenerateSingleTrajectory(task_timeout=self.generate_single_trajectory_params["timeout_s"],
                                             task_period=self.generate_single_trajectory_params["task_period"],
                                             altitude=self.generate_single_trajectory_params["altitude"],
                                             mission_file=self.generate_single_trajectory_params["mission_file"],
                                             plan_from_home=self.generate_single_trajectory_params["plan_from_home"])
        except TaskException, e:
            rospy.logwarn("Failed to generate single trajectory with error status %s" %(str(e)))
            return

        # Generate control frame switch.
        try:
            rospy.wait_for_service('switch_control_frame', timeout=3)
            self.SwitchControlFrame = rospy.ServiceProxy('switch_control_frame', SetBool)
        except TaskException, e:
            rospy.logwarn("Failed to create control frame switch, error status %s" %(str(e)))
            return

        # Logging directory
        import os
        import datetime
        stamp = datetime.datetime.utcnow()
        folder_str = "{:04d}{:02d}{:02d}{:02d}{:02d}{:02d}".format(stamp.year, stamp.month, stamp.day, stamp.hour, stamp.minute, stamp.second)
        path = os.path.expanduser("~") + '/bags/' + folder_str
        self.postprocessing_params["directory"] = rospy.get_param("~postprocessing/directory", path)
        if not os.path.exists(path):
            os.makedirs(path)

        # Start sensor recording.
        if self.record_sensors_param:
            try:
                self.tc.Logging(task_timeout=self.default_timeout_s,
                                directory=self.postprocessing_params["directory"],
                                record_cam=self.record_cam_param,
                                foreground=False)
                # Sleep 2 seconds to make sure sensor recording started at rest.
                rospy.sleep(2.0)
            except TaskException, e:
                rospy.logwarn("Failed to start sensor recording with error status %s" %(str(e)))
                return

        # Takeoff.
        try:
            self.tc.TakeOff(task_timeout=self.take_off_params["timeout_s"])
        except TaskException, e:
            rospy.logwarn("Failed take off with error status %s" %(str(e)))
            return

        # Move to trajectory start.
        if self.transition_flag_param:
            # Do high altitude transition
            # Go to transition altitude
            try:
                self.tc.Waypoint(task_timeout=self.waypoint_params["timeout_s"],
                                 goal_type=2,
                                 goal_z=self.generate_single_trajectory_params["transition_altitude"],
                                 ball_radius=self.waypoint_params["ball_radius"],
                                 terminal_vel=self.waypoint_params["terminal_vel"])
            except TaskException, e:
                rospy.logwarn("Failed to ascent to transition altitude with error status %s" %(str(e)))
                return

            #Translate to the mission start point
            try:
                self.tc.Waypoint(task_timeout=self.waypoint_params["timeout_s"],
                                 goal_type=3,
                                 goal_z=self.generate_single_trajectory_params["transition_altitude"],
                                 ball_radius=self.waypoint_params["ball_radius"],
                                 terminal_vel=self.waypoint_params["terminal_vel"])
            except TaskException, e:
                rospy.logwarn("Failed to transition to mission start point with error status %s" %(str(e)))
                return

            #Altitude control frame change (--> check if this is possible in air)
            if self.switch_control_frame_param:
                try:
                    resp = self.SwitchControlFrame(True)
                    if resp.success:
                        rospy.loginfo("Switched control frame!")
                    else:
                        rospy.logwarn("Failed to switch control frame.")
                        return
                except TaskException, e:
                    rospy.logwarn("Failed switching control frame, error status %s" %(str(e)))
                    return

            # Go (down) to trajectory altitude.
            try:
                self.tc.Waypoint(task_timeout=self.waypoint_params["timeout_s"],
                                 goal_type=4,
                                 ball_radius=self.waypoint_params["ball_radius"],
                                 terminal_vel=self.waypoint_params["terminal_vel"])
            except TaskException, e:
                rospy.logwarn("Failed to descent to mission altitude with error status %s" %(str(e)))
                return
        else:
            # Go to take off altitude.
            try:
                self.tc.Waypoint(task_timeout=self.waypoint_params["timeout_s"],
                                 goal_type=2,
                                 goal_z=self.generate_single_trajectory_params["altitude"],
                                 ball_radius=self.waypoint_params["ball_radius"])
            except TaskException, e:
                rospy.logwarn("Failed to ascent to take off altitude with error status %d %s" %(e.status, str(e)))
                return

            # Switch control frame.
            if self.switch_control_frame_param:
                try:
                    resp = self.SwitchControlFrame(True)
                    if resp.success:
                        rospy.loginfo("Switched control frame!")
                    else:
                        rospy.logwarn("Failed to switch control frame.")
                        return
                except TaskException, e:
                    rospy.logwarn("Failed switching control frame, error status %s" %(str(e)))
                    return

        # Start radar
        if self.record_radar_param:
            r = rospkg.RosPack()
            radar_script = r.get_path('fm_radar_driver') + '/src/fm_radar_driver/Radar-Python-Interface/start.py'
            radar_proc = subprocess.Popen(["python3", radar_script, self.postprocessing_params["directory"]])

        # Execute trajectory.
        try:
            self.tc.ExecuteSingleTrajectory(task_timeout=self.execute_single_trajectory_params["timeout_s"],
                                            terminal_vel=self.execute_single_trajectory_params["terminal_vel"])
        except TaskException, e:
            rospy.logwarn("Failed trajectory execution error status %s" %(str(e)))
            if self.record_radar_param:
                radar_proc.terminate()
            return

        # Stop radar.
        if self.record_radar_param:
            radar_proc.terminate()

        # Do high altitude home transition if necessary.
        if self.transition_flag_param:
            # Switch Control Frame again:
            if self.switch_control_frame_param:
                try:
                    resp = self.SwitchControlFrame(False)
                    if resp.success:
                        rospy.loginfo("Switched control frame!")
                    else:
                        rospy.logwarn("Failed to switch control frame.")
                        return
                except TaskException, e:
                    rospy.logwarn("Failed switching control frame, error status %s" %(str(e)))
                    return

            # Go to transition altitude
            try:
                self.tc.Waypoint(task_timeout=self.waypoint_params["timeout_s"],
                                 goal_type=2,
                                 goal_z=self.generate_single_trajectory_params["transition_altitude"],
                                 ball_radius=self.waypoint_params["ball_radius"],
                                 terminal_vel=self.waypoint_params["terminal_vel"])
            except TaskException, e:
                rospy.logwarn("Failed to ascent to transition altitude with error status %s" %(str(e)))
                return

            # Go to home point and stay on transition altitude
            try:
                self.tc.Waypoint(task_timeout=self.waypoint_params["timeout_s"],
                                 goal_type=5,
                                 goal_z=self.generate_single_trajectory_params["transition_altitude"],
                                 ball_radius=self.waypoint_params["ball_radius"],
                                 terminal_vel=self.waypoint_params["terminal_vel"])
            except TaskException, e:
                rospy.logwarn("Failed to move to home point with error status %s" %(str(e)))
                return

            # Switch to AGL control.
            try:
                resp = self.SwitchControlFrame(True)
                if resp.success:
                    rospy.loginfo("Switched control frame!")
                else:
                    rospy.logwarn("Failed to switch control frame.")
                    return
            except TaskException, e:
                rospy.logwarn("Failed switching control frame, error status %s" %(str(e)))
                return

            # Go (down) to takeoff altitude.
            try:
                self.tc.Waypoint(task_timeout=self.waypoint_params["timeout_s"],
                                 goal_type=5,
                                 goal_z=self.generate_single_trajectory_params["altitude"],
                                 ball_radius=self.waypoint_params["ball_radius"],
                                 terminal_vel=self.waypoint_params["terminal_vel"])
            except TaskException, e:
                rospy.logwarn("Failed to descent to takeoff altitude with error status %s" %(str(e)))
                return

            # Back into ENU control for completeness.
            try:
                resp = self.SwitchControlFrame(False)
                if resp.success:
                    rospy.loginfo("Switched control frame!")
                else:
                    rospy.logwarn("Failed to switch control frame.")
                    return
            except TaskException, e:
                rospy.logwarn("Failed switching control frame, error status %s" %(str(e)))
                return
        else:
            # otherwise change back to gps altitude for the sake of completeness.
            if self.switch_control_frame_param:
                try:
                    resp = self.SwitchControlFrame(False)
                    if resp.success:
                        rospy.loginfo("Switched control frame!")
                    else:
                        rospy.logwarn("Failed to switch control frame.")
                        return
                except TaskException, e:
                    rospy.logwarn("Failed switching control frame, error status %s" %(str(e)))
                    return

        # Land.
        try:
            self.tc.Landing(task_timeout=self.landing_params["timeout_s"])
        except TaskException, e:
            rospy.logwarn("Failed to land with error status %s" %(str(e)))
            return

        # Run post processing.
        if self.run_localization_optimization_param:
            try:
                self.tc.Postprocessing(task_timeout=self.postprocessing_params["timeout_s"],
                                topics=self.postprocessing_params["topics"],
                                directory=self.postprocessing_params["directory"])
            except TaskException, e:
                rospy.logwarn("Failed to compute batch solution with error status %s" %(str(e)))
                return

        rospy.loginfo("Mission completed")

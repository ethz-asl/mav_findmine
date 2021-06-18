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

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer, Signal

from sensor_watchdog import WatcherManager
from trajectory_button import TrajectoryButton
from ui_manager import create_head_widget


# written according to: http://wiki.ros.org/rqt/Tutorials/Writing%20a%20Python%20Plugin

class SensorWatchdogPlugin(Plugin):
    def __init__(self, context):
        super(SensorWatchdogPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('SensorWatchdogPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self.main_widget = create_head_widget()

        if context.serial_number() > 1:
            self.main_widget.setWindowTitle(self.main_widget.windowTitle() + (' (%d)' % context.serial_number()))
            rospy.logwarn("context serial_number" + str(updateTime))

        #Initialize observer
        self.watcherManager = WatcherManager(self.main_widget)

        # Add widget to the user interface
        self.traj_button = TrajectoryButton(self.main_widget.pushButton)

        #map timer to watcher update
        self.set_update_timer(rospy.get_param("~update_rate"), self.update_status) #secs

        # map it to the ui
        context.add_widget(self.main_widget)

    def set_update_timer(self, updateTime, timer_func):
        timer = QTimer(self)
        timer.timeout.connect(timer_func)
        timer.start(updateTime * 1000)
        rospy.loginfo("set refresh timer to " + str(updateTime) + " [s]")

    def update_status(self):
        self.watcherManager.update_status()
        self.traj_button.service_available()

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.watcherManager.shutdown_WatcherManager()
        #rospy.loginfo("shutdown called")

        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

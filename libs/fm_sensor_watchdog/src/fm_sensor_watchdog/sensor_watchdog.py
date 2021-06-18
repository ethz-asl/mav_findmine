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
import numpy as np
import rostopic
#msgs
import sensor_msgs.msg
import versavis.msg
import libsbp_ros_msgs.msg

import ui_manager

class WatcherManager():
    # managing watcher objects for each topic set up in the config
    #  loads ros params
    #  assigns the wigets (advanced widgets when msg content is asked)
    #  calls the update for each topic

    def __init__(self, widget):

        topicInfoList = self.load_params()
        self.watchdog_list = list()

        #fill the watchdog list
        for (topic, topic_name, value) in topicInfoList:
            #configure the watchdog
            currTopic = Watcher(topic, subscriber_duration=self.subscriber_duration)
            topicstate_layout = None

            #create column widget/layout and add it to list widget
            topic_name = rospy.get_namespace() + topic_name
            if value == "-":
                topicstate_layout = ui_manager.add_row_widget(
                    topic_name, widget.list_widget.boxCollectionLayout)
            else:
                topicstate_layout, topicinfo_label = ui_manager.add_row_widget_advanced(
                    topic_name, widget.list_widget.boxCollectionLayout)
                currTopic.info_label = topicinfo_label
                currTopic.msg_member_name = value

            # make it easy accessible for updates
            currTopic.state_layout = topicstate_layout
            #append the overall watchdog list
            self.watchdog_list.append(currTopic)

        # add the layout
        widget.list_widget.setLayout(
            widget.list_widget.boxCollectionLayout)

    def load_params(self):
        # loading update params
        self.subscriber_duration = rospy.get_param("~subscriber_duration")

        #loading topic names
        topic_string = rospy.get_param("~topic_names")
        topic_name_string = rospy.get_param("~topic_names_short")
        topic_member_string = rospy.get_param("~topic_member")

        def extract_list(sample_string):
            sample = sample_string.split(',')
            sample =  [item.strip() for item in sample]
            return np.array(sample).reshape(len(sample), 1)

        topics = extract_list(topic_string)
        topic_names = extract_list(topic_name_string)
        topic_members = extract_list(topic_member_string)

        try:
            topic_info = np.concatenate((topics, topic_names, topic_members), axis=1)

        except Exception:
            rospy.logerror("topic lists missmatch!" + "\n" +
                    "   make sure topic names, short names and members match")
            return None

        return topic_info

    def update_status(self):
        for watchdog_object in self.watchdog_list:
            watchdog_object.subscribe()

        rospy.sleep(self.subscriber_duration)

        for watchdog_object in self.watchdog_list:
            watchdog_object.check_sanity()
            watchdog_object.unsubscribe()

    def shutdown_WatcherManager(self):
        for item in self.watchdog_list:
            item.shutdown()


class Watcher():
    # watcher observes a topic assigned by the watcher manager
    #  contains a callback for each message receipt
    #  contains a timed callback (check_sanity) for rate updates
    #  subscribing and unsubscribing for each timed callback to reduce traffic
    #  the primary labels and layouts connected inside the class for easy access

    def __init__(self, topic_name, subscriber_duration = 2): # rate [Hz], max_pub_time [s]
        self.subscriber_duration = float(subscriber_duration)
        self.num_received = 0.0
        self.publish_rate = -1.0
        self.subscriber = None
        self.is_subscribed = False

        self.state_layout = None
        self.info_label = None

        self.topic_name = rospy.get_namespace() + topic_name
        self.topic_class = None
        self.msg_member_name = None
        self.msg_content = None

        self.check_topic()

    def check_topic(self):
        try:
            self.topic_class, _, _ = rostopic.get_topic_class(self.topic_name)
        except Exception as exc:
            self.topic_class = None

        if self.topic_class is None:
            rospy.logwarn("Topic " + self.topic_name + " is not advertised yet!")
            return False
        else:
            return True

    def sensor_callback(self, data): #reset timer with every topic publish
        self.num_received += 1.0
        self.publish_rate = min(self.num_received / self.subscriber_duration, 1000.0)
        if self.info_label is not None:
            self.msg_content = eval("data." + self.msg_member_name)

    def subscribe(self):
        self.num_received = 0
        self.publish_rate = -1.0

        # when topic wasn't advertised so far, check if it is now
        if self.topic_name is None:
            self.check_topic()

        # when topic is available
        if self.topic_class is not None:
            self.subscriber = rospy.Subscriber(self.topic_name, self.topic_class, self.sensor_callback)
            self.is_subscribed = True

    def unsubscribe(self):
        if self.is_subscribed:
            self.subscriber.unregister()
            self.is_subscribed = False

    def check_sanity(self):
        # when msgs available, set label to green, set publish rate,
        # get msg_content when asked
        if self.num_received > 0:
            ui_manager.set_layout_state(self.state_layout, "green",
                freq = round(self.publish_rate,1))
            if self.msg_content is not None:
                ui_manager.set_info_label(self.info_label, self.msg_member_name, self.msg_content)
        else:
            ui_manager.set_layout_state(self.state_layout, "red")
        return

    def shutdown(self):
        if self.is_subscribed:
            self.subscriber.unregister()
            self.is_subscribed = False

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

from python_qt_binding.QtWidgets import QWidget, QLabel, QVBoxLayout,\
    QHBoxLayout, QStackedLayout, QPushButton

import labels

def create_head_widget():
    widget = QWidget()

    top_VBox_layout = QVBoxLayout()

    widget.status_label = QLabel()
    widget.status_label.setText("Sensor status:")

    widget.traj_label = QLabel()
    widget.traj_label.setText("Trajectory operations:")

    widget.list_widget = QWidget()
    widget.list_widget.boxCollectionLayout = QVBoxLayout()

    widget.pushButton = QPushButton("pushButton")
    #widget.pushButton.setFixedSize(300, 20)

    widget.list_widget_button = QWidget()
    widget.list_widget_button.boxCollectionLayout = QVBoxLayout()
    widget.list_widget_button.boxCollectionLayout.addWidget(widget.pushButton)
    widget.list_widget_button.setLayout(widget.list_widget_button.boxCollectionLayout)

    top_VBox_layout.addWidget(widget.status_label)
    top_VBox_layout.addWidget(widget.list_widget)
    top_VBox_layout.addWidget(widget.traj_label)

    top_VBox_layout.addWidget(widget.list_widget_button)
    top_VBox_layout.addStretch()


    widget.setLayout(top_VBox_layout)
    widget.setObjectName('Form')
    return widget



def add_row_widget(topic, target):
    topicBoxWidget = QWidget()
    topicBoxLayout = QHBoxLayout()

    #configure text label
    topicNameLabel = QLabel()
    topicNameLabel.setText(topic)
    topicNameLabel.setFixedSize(200, 20)
    topicNameLabel.setStyleSheet('background-color: lightgray')
    topicNameLayout = QStackedLayout()
    topicNameLayout.addWidget(topicNameLabel)
    topicNameLayout.setCurrentIndex(0)
    topicNameWidget = QWidget()
    topicNameWidget.setLayout(topicNameLayout)

    #configure state label
    topicStateWidget = QWidget()
    topicStateWidget.setFixedSize(100,40)
    topicStateLayout = labels.getStackedColorLabels()

    topicStateLayout.setCurrentIndex(1) #labels should become orange
    topicStateWidget.setLayout(topicStateLayout)

    #add topic to column layout
    topicBoxLayout.addWidget(topicStateWidget)
    topicBoxLayout.addWidget(topicNameWidget)
    topicBoxLayout.addStretch()

    #set here nonspreading!!

    #set row layout
    topicBoxWidget.setLayout(topicBoxLayout)
    target.addWidget(topicBoxWidget)

    return topicStateLayout

def add_row_widget_advanced(topic, target):
    topicBoxWidget = QWidget()
    topicBoxLayout = QHBoxLayout()

    #configure text label with info
    topicNameLabel = QLabel()
    topicNameLabel.setText(topic)
    topicNameLabel.setFixedSize(200, 20)
    topicNameLabel.setStyleSheet('background-color: lightgray')
    topicInfoLabel = QLabel()
    topicInfoLabel.setText("-")
    #topicInfoLabel.setFixedSize(200, 20)
    #topicInfoLabel.setStyleSheet('background-color: lightgray')

    topicNameLayout = QVBoxLayout()
    topicNameLayout.setContentsMargins(0, 0, 0, 0)
    topicNameLayout.addWidget(topicNameLabel)
    topicNameLayout.addWidget(topicInfoLabel)
    topicNameLayout.addStretch()

    topicNameWidget = QWidget()
    topicNameWidget.setLayout(topicNameLayout)

    #configure state label
    topicStateWidget = QWidget()
    topicStateWidget.setFixedSize(100,40)
    topicStateLayout = labels.getStackedColorLabels(double_size = True)
    topicStateLayout.setCurrentIndex(1) #labels should become orange
    topicStateWidget.setLayout(topicStateLayout)

    #add topic to column layout
    topicBoxLayout.addWidget(topicStateWidget)
    topicBoxLayout.addWidget(topicNameWidget)
    topicBoxLayout.addStretch()

    #set here nonspreading!!

    #set row layout
    topicBoxWidget.setLayout(topicBoxLayout)
    target.addWidget(topicBoxWidget)

    return topicStateLayout, topicInfoLabel


def set_layout_state(layout, color, freq = None):
    if color == "green":
        layout.setCurrentIndex(0)
        layout.currentWidget().setText("OK   (" +str(freq) + " Hz)")

    elif color == "red":
        layout.setCurrentIndex(2)
    else:
        layout.setCurrentIndex(1)
    return layout

def set_info_label(topicInfoLabel, topic_type, info):
    if type(info) is tuple:
        tmp_info = list()
        for element in info:
            if type(element) is float:
                tmp_info.append(round(element, 2))
        info = tmp_info
    elif type(info) is float:
        element = round(element, 2)
    topicInfoLabel.setText(str(topic_type) +": " +str(info))

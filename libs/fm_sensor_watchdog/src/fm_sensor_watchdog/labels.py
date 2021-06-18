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

from python_qt_binding.QtWidgets import QLabel, QStackedLayout

class colorLabels:
    def __init__(self, double_size = False):
        statusSet = [['OK', 'green'], ['waiting', 'yellow'], ['missing', 'red']]
        colorSet = ['green', 'yellow', 'red']
        labelList = list()
        for status in statusSet:
            statusLabel = QLabel()
            statusLabel.setText(status[0])
            if double_size:
                statusLabel.setFixedSize(100, 40)
            else:
                statusLabel.setFixedSize(100, 20)

            statusLabel.setStyleSheet('background-color: ' + status[1])
            labelList.append(statusLabel)

        self.statusSet = labelList
        self.ok = labelList[0]
        self.waiting = labelList[1]
        self.missing = labelList[2]

def getStackedColorLabels(double_size = False): #0 --> ok, 1 --> waiting, 2 --> missing
    labelTriple = colorLabels(double_size)
    stackedColorLabel = QStackedLayout()

    for item in labelTriple.statusSet:
        stackedColorLabel.addWidget(item)
    return stackedColorLabel

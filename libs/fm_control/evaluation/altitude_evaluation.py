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

import rosbag_pandas
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

df = rosbag_pandas.bag_to_dataframe('2018-12-03-17-46-44_controls.bag')
for c in df.columns:
    print c, df[c].min(),  df[c].max()

namespace='moa_'

# Plot altitude.
z_ulanding = df['moa_ulanding_range__range']
z_ulanding = z_ulanding.dropna()
half_page_width = (5.0,5.0)
fig, ax = plt.subplots(figsize=half_page_width)
fig.patch.set_facecolor('white')
plt.xlabel("Time")
plt.ylabel("uLanding AGL [m]")
z_ulanding.plot()
#plt.plot(t_ulanding, z_ulanding, color='red', linewidth=2, linestyle='-')
ax.grid(color="k", alpha=0.1, linewidth=1, linestyle="-")
plt.show()

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

import numpy as np
import matplotlib.pyplot as plt
import gdal
from osgeo.osr import SpatialReference, CoordinateTransformation
import argparse

parser = argparse.ArgumentParser(description='Evaluate PPK solution.')
parser.add_argument('files', metavar='IN_FILES', help='.pos files to process')
parser.add_argument('enu_lat', help='ENU lat for comparison', type=float,)
parser.add_argument('enu_lon', help='ENU lon for comparison', type=float,)
parser.add_argument('enu_alt', help='ENU alt for comparison', type=float,)
args = parser.parse_args()

form = ["rx", "bx", "gx"]
iform = 0
enu_origin_wgs84 = np.array([args.enu_lat, args.enu_lon, args.enu_alt])
print("East origin: {}".format(enu_origin_wgs84[0]))
print("North origin: {}".format(enu_origin_wgs84[1]))
print("Up origin: {}".format(enu_origin_wgs84[2]))

wgs84 = SpatialReference()
wgs84.ImportFromEPSG(4326)
enu = SpatialReference()
enu.SetWellKnownGeogCS("WGS84")
enu.SetOrthographic(enu_origin_wgs84[0], enu_origin_wgs84[1], 0.0, 0.0)
enu_altitude = enu_origin_wgs84[2]
latlon2en = CoordinateTransformation(wgs84, enu)

# plot points based on quality
plt.figure()

files = [ args.files ]

for f in files:

    print("File = "+str(f))

    data = np.genfromtxt(f, comments="%",
	         	             dtype=['str', 'str', 'double', 'double', 'double', 'int', 'int', 'double', 'double', 'double',
		                    'double', 'double', 'double', 'double', 'double'])

	# generate simple numpy array from this
    data_np = np.array([[row[2],row[3],row[4],row[5],row[6]] for row in data]) # lat, lon, alt, Q, ns

    data_q1_np = data_np[data_np[:,3] == 1, :]
    data_qx_np = data_np[data_np[:,3] != 1, :]

    print("Number of Q1 Fixes of data  = "+ str(len(data_q1_np)))



    avg_q1 = np.mean(data_q1_np, axis=0)
    std_q1 = np.std(data_q1_np, axis =0)
    print("Data 1 Q1 Average = ")
    print("Lat: {}".format(avg_q1[0]))
    print("Lon: {}".format(avg_q1[1]))
    print("Alt: {}".format(avg_q1[2]))


	#print("Distance: {}".format(np.linalg.norm(np.array([avg_q1[0], avg_q1[1], avg_q1[2]]) - ecef_compare)))



    plt.plot(data_q1_np[:,0], data_q1_np[:,1], form[iform],  alpha=0.5)
    plt.plot(avg_q1[0], avg_q1[1], 'kx')
    plt.title("PPK solution")
    plt.xlabel("Latitude")
    plt.ylabel("Longitude")
    iform = iform +1

	# Calculate ENU offset from origin.
    point_enu = latlon2en.TransformPoint(avg_q1[1], avg_q1[0], avg_q1[2])
    up = point_enu[2] - enu_altitude
    print("East: {}".format(point_enu[0]))
    print("North: {}".format(point_enu[1]))
    print("Up: {}".format(up))
    print("Offset: {}".format(np.linalg.norm(np.array([point_enu[0], point_enu[1], up]))))



plt.show()


# echo " 7.73199382297 46.0014420174  1931.19731802" | gdaltransform -s_srs WGS84 -t_srs EPSG:32632
#
# Pos: 401818.166603165 5094989.30112777 1931.19731802


# echo " 7.73199372593 46.0014422246  1931.23522945" | gdaltransform -s_srs WGS84 -t_srs EPSG:32632
# Pos: 401818.159455947 5094989.32426857 1931.23522945

# Error total in R^3 : 0.0449 m
# Error horizontaly: 0.0242 m
# Error verticaly: 0.038 cm

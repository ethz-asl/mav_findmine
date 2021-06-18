#!/usr/bin/env python

# MIT License
#
# Copyright (c) 2020 Nicholas Lawrance, Lucas Streichenberg, Thomas Mantel, ASL, ETH Zurich, Switzerland
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

# PYTHON_ARGCOMPLETE_OK

## extract rgb images from rosbags

# v3.0  - 2020-08-21 stlucas:
#   - reduced to rgb extraction for findmine use
#   - embedding in findmine stack
# v2.0  - 2019-08-20 lawrancn:
#   - changed to object-oriented so can import same reader from elsewhere
# v1.1  - 2016-08-12 mantelt:
#   - use milliseconds in filename
#   - add geotag based on interpolated position (using rosbag time)
# v1.0  - 2016-08-09 mantelt:
#   first release

import rosbag
import argparse
import datetime
import cv_bridge
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import piexif
import time
import csv
from scipy.spatial.transform import Rotation, Slerp
from math import radians, cos, sin, asin, sqrt

from PIL import Image
from scipy.interpolate import interp1d

from fm_pix4d.path_helper import read_in_path, create_dir, norm_path

plt.rc('font',**{'family':'serif','sans-serif':['Computer Modern Roman']})
plt.rc('text', usetex=True)

def main(args):
    inputfiles = read_in_path(args.path_in)

    if inputfiles is None or len(inputfiles) < 1:
        print(('specified input path neither is a file nor a directory' +
        'containing bag files.'))
        print('exiting...')
        return -1

    for bag in inputfiles:
        if len(inputfiles) > 1:
            path_out = os.path.join(args.path_out, os.path.splitext(os.path.basename(bag))[0])
        else:
            path_out = args.path_out
        bag_reader = RGBRosbagReader(bag, path_out, min_dist = args.min_dist)

        if args.add_geotag:
            if args.base_ground_truth_path is not None and args.base_sampled_path is not None:
                bag_reader.find_position_error(args.base_ground_truth_path,
                    args.base_sampled_path)

            bag_reader.build_position_interpolates(args.pos_topic)

        if args.angle_limit > 0:
            bag_reader.build_quat_interpolates(args.pose_topic)

        bag_reader.write_rgb(args.rgb_topic,
                             out_dir=args.rgb_out_dir,
                             geotag=args.add_geotag, max_angle=args.angle_limit)

    print('All images processed.')



class RGBRosbagReader(object):
    """
    Class object to open a rosbag with RGB and/or IR images.

    ...

    Methods
    -------

    build_position_interpolates(pos_topic)
        Build interpolates to access position (lat/lon/alt) data by time

    write_rgb(rgb_topic, out_dir=None, geotag=False):
        Write RGB images from specified topic, with optional exif geotags.
        Default output dir is ./rgb

    """

    _lat_interp = None
    _lon_interp = None
    _alt_interp = None
    _dlat_lon_alt = None
    _quat_interp = None
    _state_interp = None


    def __init__(self, bagfile, path_out=None, pos_topic=None, min_dist=None):
        self.path_out = path_out

        status = SimpleCounter('Opening {0}...'.format(os.path.basename(bagfile)))
        self.bag = rosbag.Bag(bagfile)
        status.close()

        if pos_topic is not None:
            self.build_position_interpolates(pos_topic)

        self.bridge = cv_bridge.CvBridge()
        self.min_dist = min_dist

    def _check_topic(self, topic_name, msg_type):
        """Check the topic is in the bag and has the correct message type."""
        types, topic_info = self.bag.get_type_and_topic_info(topic_name)
        assert topic_info[topic_name].msg_type == msg_type
        message_count = topic_info[topic_name].message_count
        assert message_count > 0
        return message_count

    def _build_output_dir(self, out_dir=None, output_extension=None):
        if out_dir is not None:
            dir_out = create_dir(out_dir)
        elif output_extension is not None:
            dir_out = create_dir(os.path.join(self.path_out, output_extension))
        else:
            dir_out = create_dir(self.path_out)
        return dir_out

    def _get_exif(self, t):
        lat, lon, alt = self._get_lat_lon_alt(t)
        secs = t.to_sec()
        return create_exif(secs, float(lat), float(lon), float(alt))

    def _get_lat_lon_alt(self, t):
        lat = self._lat_interp(t.to_nsec())
        lon = self._lon_interp(t.to_nsec())
        alt = self._alt_interp(t.to_nsec())
        return lat, lon, alt

    def build_position_interpolates(self, pos_topic):
        """Build position interpolate objects for getting lat/lon/alt by time"""
        n_poses = self._check_topic(pos_topic, 'sensor_msgs/NavSatFix')
        msg = ('Found {0} NavSatFix msgs in topic {1}, creating '
               'interpolates... ').format(n_poses, pos_topic)
        status = SimpleCounter(msg)
        timestamps = []
        lats, lons, alts = [], [], []
        for i, (topic, msg, t) in enumerate(self.bag.read_messages([pos_topic])):
            timestamps.append(t.to_nsec())
            lats.append(msg.latitude)
            lons.append(msg.longitude)
            alts.append(msg.altitude)
            status.update(i, n_poses)

        #shift positions if offset exists

        if self._dlat_lon_alt is not None:
            lats = [lat + self._dlat_lon_alt[0] for lat in lats]
            lons = [lon + self._dlat_lon_alt[1] for lon in lons]
            alts = [alt + self._dlat_lon_alt[2] for alt in alts]
            print(" Corrected position with ground truth")

        # Build interpolates. Fill_values will be used as extrapolated values
        #  outside each end
        self._lat_interp = interp1d(timestamps, lats, bounds_error=False,
                                    fill_value=(lats[0], lats[-1]))
        self._lon_interp = interp1d(timestamps, lons, bounds_error=False,
                                    fill_value=(lons[0], lons[-1]))
        self._alt_interp = interp1d(timestamps, alts, bounds_error=False,
                                    fill_value=(alts[0], alts[-1]))
        status.close()

    def find_position_error(self, base_true, base_sampled):

        base_sampled_coordinates = extract_coordinates_from_file(base_sampled, ["lat_wgs84", "lon_wgs84", "alt_wgs84"])
        base_true_coordinates = extract_coordinates_from_file(base_true, ["Lat", "Lon", "Alt"])
        self._dlat_lon_alt = list()

        self._dlat_lon_alt.append(base_true_coordinates[0] - base_sampled_coordinates[0])
        self._dlat_lon_alt.append(base_true_coordinates[1] - base_sampled_coordinates[1])
        self._dlat_lon_alt.append(base_true_coordinates[2] - base_sampled_coordinates[2])
        print("found offset to ground trougth:")
        print(self._dlat_lon_alt)

    def display_path(self, image_topic):
        lats, lons = [], []
        n_images = self._check_topic(image_topic, 'sensor_msgs/Image')

        msg = 'Reading {0} images from topic {1}...'.format(n_images, image_topic)
        status = SimpleCounter(msg)
        for i, (topic, msg, t) in enumerate(self.bag.read_messages(image_topic)):
            lat, lon, alt = self._get_lat_lon_alt(t)
            lats.append(lat)
            lons.append(lon)
            status.update(i+1, n_images)
        status.close()

        def onclick(event):
            global ix, iy
            ix, iy = event.xdata, event.ydata
            print('x = %f, y = %f'%(ix, iy))

            global coords
            coords = [ix, iy]

            return coords

        fig = plt.figure()
        plt.plot(lats, lons)
        plt.plot(lats, lons, 'x')
        fig.canvas.mpl_connect('button_press_event', onclick)
        plt.show()

    def build_quat_interpolates(self, pose_topic):
        """Build quaternion interpolate objects for getting orientation by time"""
        n_poses = self._check_topic(pose_topic, 'geometry_msgs/PoseStamped')
        msg = ('Found {0} PoseStamped msgs in topic {1}, creating '
               'interpolates... ').format(n_poses, pose_topic)
        status = SimpleCounter(msg)
        timestamps = []
        quaternion_poses = []
        for i, (topic, msg, t) in enumerate(self.bag.read_messages([pose_topic])):
            timestamps.append(t.to_nsec())
            q = msg.pose.orientation
            quaternion_poses.append([q.x, q.y, q.z, q.w])
            status.update(i, n_poses)

        # Build interpolates. Fill_values will be used as extrapolated values
        #  outside each end
        self._quat_interp = Slerp(timestamps, Rotation.from_quat(quaternion_poses))
        status.close()

    def build_state_interpolates(self, state_topic):
        n_poses = self._check_topic(state_topic, 'mavros_msgs/State')
        msg = ('Found {0} State msgs in topic {1}, creating '
               'interpolates... ').format(n_poses, state_topic)
        status = SimpleCounter(msg)
        timestamps = []
        states = ['NULL']
        for i, (topic, msg, t) in enumerate(self.bag.read_messages([state_topic])):
            if msg.mode != states[-1]:      # to prevent saving unnecessary non-changes in state
                timestamps.append(t.to_nsec())
                states.append(msg.mode)
            status.update(i, n_poses)

        # Build interpolates. Fill_values will be used as extrapolated values
        #  outside each end
        self._state_interp = (np.array(timestamps), states)
        status.close()

    def get_rpy(self, t):
        msg = ('Pose requested, but no interpolates found. Run '
               'build_quat_interpolates() first!')
        assert self._quat_interp is not None, msg

        tt = np.array([ti.to_nsec() for ti in np.atleast_1d(t)])
        tt = np.clip(tt, a_min=self._quat_interp.times[0], a_max=self._quat_interp.times[-1])
        interp_rots = self._quat_interp(tt)
        return interp_rots.as_euler('xyz', degrees=True)

    def get_state(self, t):
        msg = ('State requested, but no interpolates found. Run '
               'build_state_interpolates() first!')
        assert self._state_interp is not None, msg

        tt = np.array([ti.to_nsec() for ti in np.atleast_1d(t)])
        dex = [np.sum(ti > self._state_interp[0]) for ti in tt]
        return [self._state_interp[1][i] for i in dex]


    def write_rgb(self, rgb_topic, out_dir=None, geotag=False, max_angle=-1.0):
        if geotag:
            msg = ('Geotag requested, but no interpolates found. Run '
                   'build_position_interpolates() first!')
            assert self._lat_interp is not None, msg
        if max_angle > 0:
            msg = ('Pose requested, but no interpolates found. Run '
                   'build_quat_interpolates() first!')
            assert self._quat_interp is not None, msg
        else:
            exif_bytes = '[]'

        n_rgb = self._check_topic(rgb_topic, 'sensor_msgs/Image')
        out_dir_rgb = self._build_output_dir(out_dir, 'rgb')

        status = SimpleCounter(
            'Writing RGB images to {0}...'.format(out_dir_rgb), min_step=0.01)
        lat_prev, lon_prev, alt_prev = [0.0,0.0,0.0]
        for i, (topic, msg, t) in enumerate(self.bag.read_messages(rgb_topic)):
            if max_angle > 0:
                roll, pitch, yaw = self.get_rpy(t)[0]
                if abs(roll) > max_angle or abs(pitch) > max_angle:
                    continue

            imgtime = datetime.datetime.fromtimestamp(t.secs)
            datestring = imgtime.strftime('%Y%m%d_%H%M%S') + \
                '_%03d' % (t.nsecs / 1e6)
            if geotag:
                exif_bytes = self._get_exif(t)

                #check if not to close
                lat, lon, alt = self._get_lat_lon_alt(t)
                d_hor = haversine(lon, lat, lon_prev, lat_prev)
                if self.min_dist**2 > d_hor**2:
                    continue
                lat_prev, lon_prev, alt_prev = [lat, lon, alt]

            cimage = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            im = Image.fromarray(cimage)
            filename = os.path.join(out_dir_rgb, datestring + '.jpg')
            im.save(filename, 'JPEG', quality=95, exif=exif_bytes)
            status.update(i, n_rgb)
        status.close()


class SimpleCounter(object):
    """Basic object for printing ongoing process status to stdout

    Create with opening message, optional end message,
    optional minimum step (default 5%).
    Use obj.update(progress) to update progress, obj.close() to finish
    """

    def __init__(self, start_string='Running process... ', end='done.', min_step=0.05):
        self.step = min_step
        self._current_val = min_step
        self.end = end
        self._finished = False
        sys.stdout.write('{0}{1:4d}%'.format(start_string, 0))
        sys.stdout.flush()

    def update(self, progress, max_val=None):
        # If only one input argument, assume this is progress in [0, 1.0]
        # If given two arguments, assume we are at iteration progress / max_val
        if max_val is not None:
            progress = float(progress)/max_val
        if not self._finished and progress >= 1.0:
            self.close()
        elif not self._finished and progress >= self._current_val:
            self._current_val = progress
            sys.stdout.write('\b\b\b\b\b{0:4d}%'.format(int(self._current_val * 100.0)))
            self._current_val += self.step
            sys.stdout.flush()

    def close(self):
        if not self._finished:
            sys.stdout.write('\b\b\b\b\b{0}\n'.format(self.end))
            sys.stdout.flush()
            self._finished = True

def create_exif(secs, lat, lon, alt):
    def _parse(val):
        sign = 1
        if val < 0:
            val = -val
            sign = -1
        deg = int(val)
        other = (val - deg) * 60
        minutes = int(other)
        secs = (other - minutes) * 60
        secs = long(secs * 100000)
        return (sign, deg, minutes, secs)

    timestring = time.strftime("%Y:%m:%d %H:%M:%S", time.localtime(secs))
    lon_ref = "E"
    lon_sign, lon_deg, lon_min, lon_sec = _parse(lon)
    if lon_sign < 0:
        lon_ref = "W"

    lat_ref = "N"
    lat_sign, lat_deg, lat_min, lat_sec = _parse(lat)
    if lat_sign < 0:
        lat_ref = "S"

    zeroth_ifd = {piexif.ImageIFD.Make: u"Aptina",
                  piexif.ImageIFD.Model: u"MT9V034",
                  piexif.ImageIFD.XResolution: (72, 1),
                  piexif.ImageIFD.YResolution: (72, 1),
                  piexif.ImageIFD.ImageWidth: 480,
                  piexif.ImageIFD.ImageLength: 752,
                  piexif.ImageIFD.Orientation: 8}
    exif_ifd = {piexif.ExifIFD.ExifVersion: b"\x02\x00\x00\x00",
                piexif.ExifIFD.DateTimeOriginal: timestring}
    gps_ifd = {piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
               piexif.GPSIFD.GPSLatitudeRef: lat_ref,
               piexif.GPSIFD.GPSLatitude:
                   [(lat_deg, 1), (lat_min, 1), (lat_sec, 100000)],
               piexif.GPSIFD.GPSLongitudeRef: lon_ref,
               piexif.GPSIFD.GPSLongitude:
                   [(lon_deg, 1), (lon_min, 1), (lon_sec, 100000)],
               piexif.GPSIFD.GPSAltitude: (int(alt * 1000), 1000)}
    exif_dict = {"Exif": exif_ifd, "GPS": gps_ifd}
    exif_bytes = piexif.dump(exif_dict)

    return exif_bytes
def haversine(lon1, lat1, lon2, lat2):
    """
    Calculate the great circle distance between two points
    on the earth (specified in decimal degrees)
    """
    # convert decimal degrees to radians
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

    # haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a))
    r = 6371008.8 # Mean radius of earth in meter

    return c * r

def extract_coordinates_from_file(path, names = ["lat_wgs84", "lon_wgs84", "alt_wgs84"]):
    d ={}
    path = norm_path(path)
    with open(path) as f:
        for line in f:
            elements = line.split()
            if len(elements)== 2: #standard piksi msg formatting
                (key, val) = elements
                key = key.replace(":", "") #removing special characters
            elif len(elements) == 3: #ppk formating
                (key, tmp, val) = elements
            else:
                continue #skip the line if more than two elements

            if "-" in val or "_" in val: #skip for fancy strings
                continue

            d[key] = float(val)
    return d[names[0]], d[names[1]], d[names[2]]


if __name__ == "__main__":
    main(sys.argv[1:])

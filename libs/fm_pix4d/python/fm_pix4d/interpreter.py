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

import argcomplete
import argparse

"""Basic command line interpreter to read a bag file and save images."""
def process_bag_input(argv):
    parser = argparse.ArgumentParser(description='Process images in rosbag',
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('path_in', metavar='IN',
        help='can be one of the following: \n' +
        '- path to directory containing several bag files or\n' +
        '- path to one bag file to be processed')
    parser.add_argument('path_out', metavar='OUT_DIR',
        help='path to directory where output should be saved')
    parser.add_argument('--rgb-out-dir',
        help='specify specific directory where RGB images should be stored')
    parser.add_argument('--rgb-topic', dest='rgb_topic', help='topic name of ' +
        'rgb images', default='/moa/bfly/synced/image_raw_throttled')
    parser.add_argument('--no-geotag', dest='add_geotag', action='store_false',
        help='do not use NavSatFix messages for image geotags')
    parser.add_argument('--pos-topic', dest='pos_topic',
        help='topic name containing position information (of type NavSatFix)',
        default='/moa/piksi/position_receiver_0/ros/navsatfix')
    parser.add_argument('--pose-topic', dest='pose_topic',
        help='topic name containing pose information (of type PoseStamped)',
        default='/generic_pose_topic')
    parser.add_argument('--angle-limit', default=-1.0, type=float,
        help='only output images with bank and pitch angle less than specified value')
    parser.add_argument('--cam_calibration',
        help='specify location of camera calibration parameters',
        default='~/catkin_ws/src/mav_findmine/libs/fm_pix4d/' +
            'config/camchain-imucam-2020-06-18-11-25-58.yaml')
    parser.add_argument('--gcp_path',
        help='specify location of gcp_files',
        default=None)
    parser.add_argument('--min_dist',
        type=float, default = 1.0,
        help='specify minimal required distance between images')
    parser.add_argument('--base_ground_truth_path',
        help='specify location of ppk correction file',
        default=None)
    parser.add_argument('--base_sampled_path',
        help='specify location of sampled base station position',
        default=None)

    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    print("\n")

    return args

def process_gcps(argv):
    parser = argparse.ArgumentParser(description='Process sampled gcps in rosbag',
        formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('path_in', metavar='IN',
        help='path to one of the gcp files')
    parser.add_argument('path_out', metavar='OUT_DIR',
        help='path to directory where output should be saved')
    parser.add_argument('--base_ground_truth_path',
        help='specify location of ppk correction file',
        default=None)
    parser.add_argument('--base_sampled_path',
        help='specify location of sampled base station position',
        default=None)

    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    print("\n")

    return args

def process_tile_input(argv):
    parser = argparse.ArgumentParser(description='Change TMS aligned tiles to Slippy OSM format')
    parser.add_argument('-i', '--input-dir', required=True, help='Directory of TMS (google) tiles (/{z}/{y}/{x}.png)')
    parser.add_argument('-m', '--intermediate-dir', required=True, help='Directory of intermediate (transformed) tiles')
    parser.add_argument('-o', '--output-dir', default="~/map_tiles/",
        help='Target directory for OSM output tiles (must be different from input)')
    args = parser.parse_args()

    return args

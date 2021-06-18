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

import os
from shutil import copy2
import argparse
import tqdm

# Based on https://alastaira.wordpress.com/2011/07/06/converting-tms-tile-coordinates-to-googlebingosm-tile-coordinates/

def get_int_dirs(target_dir):

    # Get all subdirectories with integer names
    for root, subdirs, files in os.walk(target_dir):
        int_dirs = []
        for subdir in subdirs:
            try:
                temp = int(subdir)
            except ValueError:
                continue
            int_dirs.append(subdir)
        return int_dirs


def main(args):
    assert args.input_dir != args.intermediate_dir, 'Input and output directories must be different'

    # Loop over zoom level directories
    for zdir in get_int_dirs(args.input_dir):
        # print('Zoom level {0}'.format(zdir))
        full_old_zdir = os.path.join(args.input_dir, zdir)
        full_new_zdir = os.path.join(args.intermediate_dir, zdir)
        if not os.path.exists(full_new_zdir):
            os.makedirs(full_new_zdir)
        y_max = 1 << int(zdir)

        for xdir in tqdm.tqdm(get_int_dirs(full_old_zdir), desc='Zoom level {0}'.format(zdir)):
            full_old_xdir = os.path.join(full_old_zdir, xdir)
            full_new_xdir = os.path.join(full_new_zdir, xdir)
            if not os.path.exists(full_new_xdir):
                os.makedirs(full_new_xdir)

            for file in os.listdir(full_old_xdir):
                if file.endswith('.png'):
                    tsm_y = file.split('.')[0]
                    try:
                        osm_y = y_max - int(tsm_y) - 1
                    except ValueError:
                        print('File {0} not properly parsed, skipping.'.format(os.path.join(full_new_xdir, file)))
                        continue
                    copy2(os.path.join(full_old_xdir, file), os.path.join(full_new_xdir, str(osm_y)+'.png'))


if __name__ == "__main__":
    main()

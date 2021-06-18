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
import subprocess

def read_in_path(in_path):
    in_path = os.path.normpath(os.path.expanduser(in_path))
    in_path = os.path.abspath(in_path)
    if not os.path.exists(in_path):
        return None
    inputfiles = list()
    if os.path.isfile(in_path):
        inputfiles.append(in_path)
        return inputfiles
    elif os.path.isdir(in_path):
        inputfile_candidates = os.listdir(in_path)
        for f in inputfile_candidates:
            tempfilepath = os.path.join(in_path, f)
            if os.path.isfile(tempfilepath) and \
            os.path.splitext(tempfilepath)[1] == '.bag':
                inputfiles.append(tempfilepath)
        inputfiles.sort()
        return inputfiles
    return None

def create_dir(path):
    path = os.path.expanduser(path)
    path = os.path.normpath(path)
    if not os.path.exists(path):
        os.makedirs(path)
        # os.makedirs(path, exist_ok=True)
    return path

def norm_path(path):
    path = os.path.normpath(os.path.expanduser(path))
    path = os.path.abspath(path)
    return path


def sync_dir(src, dest):
    print("starting syncing")
    src = norm_path(src) + "/"
    dest = norm_path(dest) + "/"
    output = subprocess.check_output(["rsync", "-aI", src, dest])
    print(output)
    print("finished syncing")

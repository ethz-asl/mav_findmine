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
from fm_pix4d.path_helper import norm_path
from fm_pix4d.image_extractor import extract_coordinates_from_file

gcp_sample_name = 'sampled_position_position_receiver'
min_sample_per_dir = 3 #min required gcps


def main(args):
    #prepare paths
    bag_path  = norm_path(args.path_in)
    target_path = norm_path(args.path_out)
    #print(target_path)

    #locate necessary paths
    if args.gcp_path is None:
        gcp_path = search_gcp(bag_path)
    else:
        gcp_path = norm_path(args.gcp_path)
        gcp_path = search_gcp(gcp_path)

    if gcp_path is None:
        print(" Could not find gcp path! Exiting")
        return 0

    output_path = os.path.normpath(os.path.join(target_path,"gcp_summary.txt"))
    print(" Take gcp's from: " + gcp_path)
    print(" Writing to path: " + output_path)

    #write gcps:
    write_gcps(gcp_path, output_path)

    if args.base_ground_truth_path is not None and args.base_sampled_path is not None:
        correct_gcps(output_path, args.base_sampled_path, args.base_ground_truth_path)

#searching for gcp files bottom up
def search_gcp(project_path):
    search_iteration = 3
    failed_searches = list()

    if os.path.isfile(project_path):
        #if it's a file, move one directory up
        project_path =  os.path.dirname(project_path)

    while search_iteration > 0:
        #print("searching gcps on :" + project_path)
        gcp_path = check_for_gcp(project_path)
        if(gcp_path is not None):
            return gcp_path
        search_iteration = search_iteration -1
        failed_searches.append(project_path)

        #walk one directory up
        project_path = os.path.dirname(project_path)
    print(" gcp search failed, tried on paths: ")
    print(failed_searches)
    return None

#walk through folder structure and search for files
def check_for_gcp(path):
    found_folder = False
    for root, dirs, files in os.walk(path):
        gcp_counter = 0
        for file in files: #check if there are
            #check of files
            if gcp_sample_name in file:
                gcp_counter = gcp_counter + 1

            #check if we have enough to say its the right folder
            if gcp_counter > min_sample_per_dir:
                found_folder = True
                break
        if found_folder:
            return root

    return None

def correct_gcps(file_path, base_sampled, base_true):
    base_sampled_coordinates = extract_coordinates_from_file(base_sampled,
        ["lat_wgs84", "lon_wgs84", "alt_wgs84"])
    base_true_coordinates = extract_coordinates_from_file(base_true,
        ["Lat", "Lon", "Alt"])

    dlat = base_true_coordinates[0] - base_sampled_coordinates[0]
    dlon = base_true_coordinates[1] - base_sampled_coordinates[1]
    dalt = base_true_coordinates[2] - base_sampled_coordinates[2]

    corrected_lines = list()
    file_path = norm_path(file_path)
    print(" reading gcps")
    with open(file_path) as f:
        for line in f:
            gcp_name, lat, lon, alt = line.split(',')
            #shifting by dX
            corrected_lines.append([gcp_name,
                float(lat) + dlat,
                float(lon) + dlon,
                float(alt) + dalt])
    print(" writing corrected gcps")
    with open(file_path.replace(".txt","_corrected.txt"), "w") as file_object:
        # Append 'hello' at the end of file
        for line in corrected_lines:
            line = [str(item) for item in line]
            writing_string = ', '.join(line)
            file_object.write(writing_string)
            file_object.write("\n")

def write_gcps(gcp_dir, outfile='gcp_summary.txt'):
    # Python replacement for existing gcp_write.sh script
    n_gcp = 0
    with open(outfile, 'wt') as fh_gcp_summary:
        for file in os.listdir(gcp_dir):
            if gcp_sample_name in file:
                lat, lon, alt = extract_coordinates_from_file(os.path.join(gcp_dir, file), ["lat_wgs84", "lon_wgs84", "alt_wgs84"])
                fh_gcp_summary.write("GCP{n}, {lat:0.15f}, {lon:0.15f}, {alt:0.15f}\n".format(n=n_gcp, lat=lat, lon=lon, alt=alt))
                n_gcp += 1
    print("Wrote {0} points to {1}.".format(n_gcp, outfile))

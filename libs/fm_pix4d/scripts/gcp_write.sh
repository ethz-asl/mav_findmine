#!/bin/bash

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

OPTIND=1

outfile="gcp_summary.txt"
n_gcp=0

usage() {
    echo -e "Usage: $0 [OPTION] gcp_dir"
    echo -e "   gcp_dir   GCP survey directory with .txt files"
    echo -e "   -o          Output file (default gcp_summary.txt)"
    echo -e "   -h          Print this help and exit"
    exit 0;
}

while getopts ":o:h" opt; do
    case "$opt" in
        o) outfile=$OPTARG ;;
        h | *)
            usage ;;
    esac
done

shift $(expr $OPTIND - 1 )

if [ "$#" -lt 1 ]; then
    echo "ERROR: No GCP files specified"
    usage
fi

if [ -f "$outfile" ]; then
    echo "Specified output file ${outfile} already exists. Exiting."
    exit 0
fi

touch $outfile

for filename in $@/*.txt; do
    lat=$( sed -n -e 's/^lat_wgs84: //p' < $filename )
    lon=$( sed -n -e 's/^lon_wgs84: //p' < $filename )
    alt=$( sed -n -e 's/^alt_wgs84: //p' < $filename )
    echo "GCP${n_gcp}, ${lat}, ${lon}, ${alt}" >> $outfile
    n_gcp=$((n_gcp+1))
done

echo "Wrote ${n_gcp} points to ${outfile}."

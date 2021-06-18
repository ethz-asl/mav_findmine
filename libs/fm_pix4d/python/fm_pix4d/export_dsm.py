#!/usr/bin/python

# MIT License
#
# Copyright (c) 2021 Rik Baehnemann, ASL, ETH Zurich, Switzerland
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

import gdal
from osgeo import osr
from osgeo import ogr
import os.path
from matplotlib import pyplot as plt
import matplotlib.patches as patches
import numpy as np
import yaml

def openCoordinates(file):
    with open(file, 'r') as stream:
        try:
            dict = yaml.safe_load(stream)
            x = dict['x_enu']
            y = dict['y_enu']
            z = dict['z_enu']
            lat = dict['lat_wgs84']
            lon = dict['lon_wgs84']
            alt = dict['alt_wgs84']
            return x, y, z, lat, lon, alt
        except yaml.YAMLError as exc:
            print(exc)

def pixel(dx,dy,dsm):
    px = dsm.GetGeoTransform()[0]
    py = dsm.GetGeoTransform()[3]
    rx = dsm.GetGeoTransform()[1]
    ry = dsm.GetGeoTransform()[5]
    x = dx*rx + px
    y = dy*ry + py
    return x,y

def plotTif(file):
    print("Opening %s" % (file))
    if os.path.isfile(file):
        dsm = gdal.Open(file)
        array = dsm.GetRasterBand(1).ReadAsArray()
        [cols, rows] = array.shape
        arr_min = np.min(array[np.where(array>-10000)])
        arr_max = array.max()
        gt = dsm.GetGeoTransform()
        extent = (gt[0], gt[0] + dsm.RasterXSize * gt[1],gt[3] + dsm.RasterYSize * gt[5], gt[3])
        plt.figure()
        im = plt.imshow(array, extent=extent, origin='upper')
        plt.clim(arr_min, arr_max)
        cb = plt.colorbar(im)
        cb.formatter.set_useOffset(False)
        cb.update_ticks()
    else:
        print("ERROR! File does not exist.")


def export_dsm(input, output, enu_origin, center, min_x, max_x, min_y, max_y):
    plotTif(input)
    # Plot center and patch
    c_x, c_y, c_z, c_lat, c_lon, c_alt = openCoordinates(center)
    bottom_left = np.array([c_x, c_y]) - np.abs(np.array([min_x, min_y]))
    rect = patches.Rectangle(bottom_left, max_x - min_x, max_y - min_y, linewidth=1, edgecolor='r', facecolor='none')
    plt.gca().add_patch(rect)
    plt.scatter(x=[c_x], y=[c_y], c='r', s=1)
    plt.show()

    # Define window in WGS84 coordinates
    dsm = gdal.Open(input)
    print(dsm.GetProjection())
    InSR = osr.SpatialReference(wkt=dsm.GetProjection())
    OutSR = osr.SpatialReference()
    OutSR.ImportFromEPSG(4326) # WGS84

    bounds_min = ogr.Geometry(ogr.wkbPoint)
    bounds_min.AddPoint(bottom_left[0], bottom_left[1])
    bounds_min.AssignSpatialReference(InSR)
    bounds_min.TransformTo(OutSR)

    bounds_max = ogr.Geometry(ogr.wkbPoint)
    bounds_max.AddPoint(bottom_left[0] + max_x - min_x, bottom_left[1] + max_y - min_y)
    bounds_max.AssignSpatialReference(InSR)
    bounds_max.TransformTo(OutSR)

    new_bounds = [bounds_max.GetX(), bounds_max.GetY(), bounds_min.GetX(), bounds_min.GetY()]
    print('New bounds are:')
    print(new_bounds)
    print('Please warp original TIF.')
    print('-te %f %f %f %f -te_srs EPSG:4326' % (bounds_min.GetX(), bounds_min.GetY(), bounds_max.GetX(), bounds_max.GetY()))

    dsm_cropped = gdal.Open(input + '_cropped')
    array = dsm_cropped.GetRasterBand(1).ReadAsArray()
    [cols, rows] = array.shape
    enu_x, enu_y, enu_z, enu_lat, enu_lon, enu_alt = openCoordinates(enu_origin)
    f = open(output, 'w')
    f.write('name,x,y,z\n')
    id = 1
    for i in range(0, cols):
        for j in range(0,rows):
            x, y = pixel(i, j, dsm_cropped)
            z = array[i,j] - enu_alt
            str = 'SURF{:06d},{:.6f},{:.6f},{:.6f}\n'.format(id, x, y, z)
            f.write(str)
            id = id + 1
    f.close()


    plotTif(input + '_cropped')
    plt.scatter(x=[c_x], y=[c_y], c='r', s=1)
    plt.show()

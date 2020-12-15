###############################################################################
# Opendrive to Apollo Map Converter
# Copyright (C) 2020 fortiss GmbH
# Authors: Chan Tin Chon, Klemens Esterle, Tobias Kessler
#
# This library is free software; you can redistribute it and/or modify it
# under the terms of the GNU Lesser General Public License as published by the
# Free Software Foundation; either version 2.1 of the License, or (at your
# option) any later version.
#
# This library is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
# for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this library; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
###############################################################################

import matplotlib.pyplot as plt
#import pandas as pd
#import geopandas


def plot_geometry(ax, geometry, color=None, marker="None", line="None"):
    """plot pointSet under geometry as line

    Arguments:
        geometry {etree._Element} -- geometry node defined in HPMap specification
        style {str} -- style of the line

    Keyword Arguments:
        color {str} -- color of the line (default: {None})
    """
    pointSet = geometry.find("pointSet")
    x = []
    y = []
    for i in range(len(pointSet)):
        x1, y1 = float(pointSet[i].get("x")), float(pointSet[i].get("y"))
        x.append(x1)
        y.append(y1)

    #df = pd.DataFrame({"x" : x, "y": y})
    #gdf = geopandas.GeoDataFrame(df, geometry=geopandas.points_from_xy(df.x, df.y))

    if True:  # color is None:
        ax.plot(x, y, marker=marker, linestyle=line)
        #gdf.plot(ax=ax, marker=marker)
    else:
        #gdf.plot(ax=ax, marker=marker, color=color)
        ax.plot(x, y, marker=marker, linestyle=line, color=color)


def mark_start_idx(ax, geometry):
    pointSet = geometry.find("pointSet")
    ax.plot(float(pointSet[0].get("x")), float(
        pointSet[0].get("y")), marker="x", color="k")


def plot_map(ax, lines):
    """plot discretized map contour, which comprises of 
       1. reference line (routeView)
       2. center line of each line (centerLine)
       3. boundaries of each laneSection (boundaries of road)
       4. border of each lane

    Arguments:
        lines {dict} -- as descripted in parse_road_lines of converter
    """
    routeView = lines["routeView"]
    centerLineGeoDicts = lines["centerLineGeoDicts"]
    boundariesList = lines["boundariesList"]
    borderGeoDicts = lines["borderGeoDicts"]

    for geo in routeView:
        plot_geometry(ax, geo, color="k", marker=".", line="-")

    for centerLineGeoDict in centerLineGeoDicts:
        for laneIdx, lane in centerLineGeoDict.items():
            if laneIdx != 0:
                for geoidx, geo in enumerate(lane):
                    plot_geometry(ax, geo, marker=".",
                                  line="--", color="silver")
                    if geoidx == 0:
                        mark_start_idx(ax, geo)

    for boundaries in boundariesList:
        for boundary in boundaries:
            for geo in boundary:
                plot_geometry(ax, geo, marker=".", line="-", color="g")

    for borderGeoDict in borderGeoDicts:
        indices = sorted(borderGeoDict.keys())[1:-1]
        for laneIdx in indices:
            if laneIdx != 0:
                for geo in borderGeoDict[laneIdx]:
                    plot_geometry(ax, geo, marker=".", line="-", color="y")


def plot_junction(ax, points):
    xs = []
    ys = []
    idx = 1
    for p in points:
        xs.append(float(p.get("x")))
        ys.append(float(p.get("y")))
        #plt.text(float(p.get("x"))+0.001, float(p.get("y"))+0.001, str(idx), fontsize=12)
        idx = idx + 1
    ax.plot(xs, ys, ".:", color="red")

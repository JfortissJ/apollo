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

from .common_utils import *

import cmath
from lxml import etree
import numpy as np
from bisect import bisect
from scipy.special import fresnel
from itertools import repeat
import pyproj


def point(x, y, z=0.0):
    """create point node (HPMap)

    Arguments:
        x {float} -- x coordinate
        y {float} -- y coordinate

    Keyword Arguments:
        z {float} -- z coordinate (default: {0})

    Returns:
        etree._Element -- parsed point node
    """
    return etree.Element("point", {"x": to_str(x), "y": to_str(y),
                                   "z": to_str(z)})


def get_segment_end_idx(sCoordinates, s):
    """calculate the index of sCoordinates which value larger than s

    Arguments:
        sCoordinates {np.array(n,)} -- list of s coordinate
        s {float} -- s coordinate

    Returns:
        {int} -- index
    """
    return bisect(sCoordinates, s)


def line_arctan(p1, p2):
    """wrapper function of arctan

    Arguments:
        p1 {numpy.ndarray(1,2)} -- a 2D point
        p2 {numpy.ndarray(1,2)} -- a 2D point
    """
    return np.arctan2(p2[1]-p1[1], p2[0]-p1[0])


def get_tangent_angle(line, sCoordinates, s):
    if s <= 0:
        p1 = line[0]
        p2 = line[1]
        return line_arctan(p1, p2)
    elif s >= sCoordinates[-1]:
        p1 = line[-2]
        p2 = line[-1]
        return line_arctan(p1, p2)
    else:
        endIdx = get_segment_end_idx(sCoordinates, s)
        if sCoordinates[endIdx] == s:
            p1 = line[endIdx-1]
            p2 = line[endIdx]
            p3 = line[endIdx+1]

            sinMean = 0.5*np.sin(line_arctan(p1, p2) +
                                 np.sin(line_arctan(p2, p3)))
            cosMean = 0.5*np.cos(line_arctan(p1, p2) +
                                 np.cos(line_arctan(p2, p3)))

            return line_arctan(sinMean, cosMean)
        else:
            p1 = line[endIdx-1]
            p2 = line[endIdx]

            return line_arctan(p1, p2)


def get_normal(line, sCoordinates, s):
    """calculate the unit normal vector at s coordinate along the line

    Arguments:
        line {numpy.ndarray(n,2)} -- a discretized line
        sCoordinates {numpy.array(n,)} -- [description]
        s {float} -- s coordinate

    Returns:
        {np.array(2,)} -- unit normal vector at s along the line
    """
    tangent = get_tangent_angle(line, sCoordinates, s)
    return np.array([np.cos(tangent+0.5*np.pi), np.sin(tangent+0.5*np.pi)])


def points_to_boundary_pointSet(geoType, pointList, polys, sCoordinates, sign, xodrproj, apolloproj):
    """placeholder for generating boundary pointSet with reference line according to geometry type

    Arguments:
        geoType {str} -- geometry type (OpenDRIVE 5.3.4.1)
        pointList {numpy.array (n,2)} -- discretized line
        polys {list} -- list of OpenDRIVE poly node (etree._Element)
        sCoordinates {numpy.array (n,)} -- list of distance of the discretized points from the starting along a line (OpenDrive geometry)
        sign {int} -- sign (+1/-1) to determine left/right boundary

    Raises:
        TypeError: not supported geometry type

    Returns:
        pointSet {etree._Element} -- pointSet node of the boundary
        length {float} -- length of the boundary
    """
    polys = [to_dict(poly) for poly in polys]
    key = "sOffset" if "sOffset" in polys[0].keys() else "s"
    polysDict = {poly[key]: to_dict(poly)
                 for poly in polys}  # use s coordinate as key

    if geoType in ["line", "arc", "spiral"]:
        return points_to_curve_boundary_pointSet(pointList, polysDict, sCoordinates, sign, xodrproj, apolloproj)
    else:
        raise TypeError("Geometry type not support: {}".format(geoType))


# def points_to_line_boundary_pointSet(pointList, offsetDict, sCoordinates, sign):
#     """generating boundary pointSet from straight reference line

#     Arguments:
#         pointList {numpy.array (n,2)} -- discretized line
#         offsetDict {dict} -- dict containing parameters of a third order polynomial function
#         sCoordinates {numpy.array (n,)} -- list of distance of the discretized points from the starting along a line (OpenDrive geometry)
#         sign {int} -- sign (+1/-1) to determine left/right boundary
#         widths {numpy.array (n,)} -- width of the current lane

#     Returns:
#         pointSet {etree._Element} -- pointSet node of the boundary
#         length {float} -- length of the boundary
#     """
#     widths = np.array([polynorm(offsetDict, s) for s in sCoordinates])

#     normal = get_normal(
#         pointList, sCoordinates[[0, -1]], sCoordinates[[0, -1]])

#     # normal = pointList[-1]-pointList[0]

#     # length = np.linalg.norm(normal)

#     # normal[0], normal[1] = normal[1], -normal[0]
#     # normal /= length
#     pointList = pointList+normal*widths[:, np.newaxis]*sign

#     pointSet = etree.Element("pointSet")
#     for p in pointList:
#         pointSet.append(point(p[0], p[1]))

#     return pointSet, pointList, length, widths


def points_to_curve_boundary_pointSet(pointList, polysDict, sCoordinates, sign, xodrproj, apolloproj):
    """generating boundary pointSet from curved reference line (spiral, arc)

    Arguments:
        pointList {numpy.array (n,2)} -- discretized line
        widthsDict {dict} -- dict containing parameters of a third order polynomial function
        sCoordinates {numpy.array (n,)} -- list of distance of the discretized points from the starting along a line (OpenDrive geometry)
        sign {int} -- sign (+1/-1) to determine left/right boundary

    Returns:
        pointSet {etree._Element} -- pointSet node of the boundary
        length {float} -- length of the boundary
        widths {numpy.array (n,)} -- width of the current lane
    """
    # calculate width with different poly function
    # according the s coodinates of the discretized line and poly function
    intervals = sorted(polysDict.keys())
    widths = []
    i = 0
    j = 0
    sOffsetVec = []

    while j != len(sCoordinates):
        while j != len(sCoordinates):
            if i != len(intervals)-1 and sCoordinates[j] >= intervals[i+1]:
                i += 1
            else:
                # see page 57, 5.3.7.2.1.1.2 Lane Width Record of the xodr spec (rev. 1.5)
                if polysDict[intervals[i]].get('sOffset') is not None:
                    sOffset = polysDict[intervals[i]].get('sOffset')
                else:
                    sOffset = 0.0
                sOffsetVec.append(sOffset)
                widths.append(
                    polynorm(polysDict[intervals[i]], sCoordinates[j]-sOffset))
                j += 1
    widths = np.array(widths)

    normals = np.array(
        list(map(get_normal, repeat(pointList), repeat(sCoordinates), sCoordinates)))
    pointList = pointList+normals*widths[:, np.newaxis]*sign

    pointSet = etree.Element("pointSet")
    for p in pointList:
        pwgs84 = pyproj.transform(
            xodrproj, apolloproj, p[0], p[1])  # convert to wgs84
        # apollo wants long/lat, flip coordinates
        pwgs84 = np.flip(pwgs84).tolist()
        pointSet.append(point(*pwgs84))

    length = np.sum(np.linalg.norm(np.diff(pointList, axis=0), 2, axis=1))

    return pointSet, pointList, length, widths


def spiral_interp(s, x, y, cDot, curvStart, hdg):
    """calculate the coordinate of endpoint of a spiral adapted from odrspiral on OpenDRIVE website

    Arguments:
        s {float} -- length of the spiral from the starting point
        x {float} -- start position
        y {float} -- start position
        cDot {float}
        curvStart {float}
        hdg {float} -- start orientation of the spiral in radian

    Returns:
        numpy.array (2,) -- coordinate of the spiral endpoint
    """
    c0 = complex(x, y)
    ci = complex(0, 1)

    xxa = (curvStart+cDot*s)/np.sqrt(np.pi*abs(cDot))
    ssa, cca = fresnel(xxa)

    xxb = curvStart/np.sqrt(np.pi*abs(cDot))
    ssb, ccb = fresnel(xxb)

    cs1 = np.sqrt(np.pi/abs(cDot))*cmath.exp(ci*(hdg-curvStart**2/2/cDot))
    cs2 = np.sign(cDot)*(cca-ccb)+ci*ssa-ci*ssb
    cs = c0+cs1*cs2

    xcoord = cs.real
    ycoord = cs.imag

    return np.array([xcoord, ycoord])


def spiral_to_points(geometry, discretizeDistance, xodrproj, apolloproj, sectionEndOffset):
    """generating discretized reference line to pointSet (HDMap) in HDMap from a spiral geometry (OpenDRIVE)

    Arguments:
        geometry {etree._Element} -- spiral geometry node of OpenDRIVE
        discretizeDistance {float} -- distance to discretized line

    Returns:
        pointSet {etree._Element} -- pointSet node of the discretized reference line
        pointList {numpy.array (n,2)} -- discretized reference line
        sCoordinates {numpy.array (n,)} -- list of distance of the discretized points from the starting along a line (OpenDrive geometry)
    """
    pointSet = etree.Element("pointSet")
    pointList = []
    sCoordinates = []

    GA = to_dict(geometry)
    curvStart = float(geometry[0].get("curvStart"))
    curvEnd = float(geometry[0].get("curvEnd"))
    cDot = (curvEnd-curvStart)/GA["length"]
    s = 0.0

    while s < GA["length"]:
        sCoordinates.append(s)
        p = spiral_interp(s, GA["x"], GA["y"], cDot, curvStart, GA["hdg"])
        pwgs84 = pyproj.transform(
            xodrproj, apolloproj, p[0], p[1])  # convert to wgs84
        # apollo wants long/lat, flip coordinates
        pwgs84 = np.flip(pwgs84).tolist()
        pointSet.append(point(*pwgs84))
        pointList.append(p)
        s += discretizeDistance

    lastS = GA["length"] - sectionEndOffset
    endPoint = spiral_interp(lastS, GA["x"],
                             GA["y"], cDot, curvStart, GA["hdg"])
    endPointwgs84 = pyproj.transform(
        xodrproj, apolloproj, endPoint[0], endPoint[1])
    # apollo wants long/lat, flip coordinates
    endPointwgs84 = np.flip(endPointwgs84).tolist()
    pointSet.append(point(*endPointwgs84))
    pointList.append(endPoint)
    sCoordinates.append(lastS + sectionEndOffset)

    return pointSet, np.array(pointList), np.array(sCoordinates)


def arc_interp(GA, length, curvature):
    """calculate the coordinate of endpoint of an arc

    Arguments:
        GA {dict} -- parsed geometry
        length {float} -- length of the arc from the starting point
        curvature {float} -- curvature of the arc

    Returns:
        numpy.array (2,) -- coordinate of the arc endpoint
    """
    hdg = GA["hdg"] % (np.pi * 2) - np.pi/2
    a = 2/curvature*np.sin(length*curvature/2)
    alpha = (np.pi-length*curvature)/2-hdg

    xcoord = GA["x"] - a*np.cos(alpha)
    ycoord = GA["y"] + a*np.sin(alpha)

    return np.array([xcoord, ycoord])


def arc_to_points(geometry, discretizeDistance, xodrproj, apolloproj, sectionEndOffset):
    """generating discretized reference line to pointSet (HDMap) in HDMap 
       from an arc geometry (OpenDRIVE)

    Arguments:
        geometry {etree._Element} -- arc geometry node of OpenDRIVE
        discretizeDistance {float} -- distance to discretized line

    Returns:
        pointSet {etree._Element} -- pointSet node of the discretized reference line
        pointList {numpy.array (n,2)} -- discretized reference line
        sCoordinates {numpy.array (n,)} -- list of distance of the discretized points from the starting along a line (OpenDrive geometry)
    """
    pointSet = etree.Element("pointSet")
    pointList = []
    sCoordinates = []

    GA = to_dict(geometry)
    s = 0.0
    curvature = float(geometry[0].get("curvature"))
    lastS = GA["length"] - sectionEndOffset

    while s < lastS:
        sCoordinates.append(s)
        p = arc_interp(GA, s, curvature)
        pwgs84 = pyproj.transform(
            xodrproj, apolloproj, p[0], p[1])  # convert to wgs84
        # apollo wants long/lat, flip coordinates
        pwgs84 = np.flip(pwgs84).tolist()
        pointSet.append(point(*pwgs84))
        pointList.append(p)
        s += discretizeDistance

    endPoint = arc_interp(GA, lastS, curvature)
    sCoordinates.append(lastS + sectionEndOffset)
    endPointwgs84 = pyproj.transform(
        xodrproj, apolloproj, endPoint[0], endPoint[1])
    # apollo wants long/lat, flip coordinates
    endPointwgs84 = np.flip(endPointwgs84).tolist()
    pointSet.append(point(*endPointwgs84))
    pointList.append(endPoint)

    return pointSet, np.array(pointList), np.array(sCoordinates)


def line_interp(GA, length):
    """calculate the coordinate of endpoint of a straight line

    Arguments:
        GA {dict} -- parsed geometry
        length {float} -- length of the arc from the starting point

    Returns:
        numpy.array (2,) -- coordinate of the line endpoint
    """
    x = GA["x"]+np.cos(GA["hdg"])*length
    y = GA["y"]+np.sin(GA["hdg"])*length
    return np.array([x, y])


def line_to_points(geometry, discretizeDistance, xodrproj, apolloproj, sectionEndOffset):
    """generating discretized reference line to pointSet (HDMap) in HDMap from a straight line geometry (OpenDRIVE)

    Arguments:
        geometry {etree._Element} -- straight line geometry node of OpenDRIVE
        discretizeDistance {float} -- distance to discretized line

    Returns:
        pointSet {etree._Element} -- pointSet node of the discretized reference line
        pointList {numpy.array (n,2)} -- discretized reference line
        sCoordinates {numpy.array (n,)} -- list of distance of the discretized points from the starting along a line (OpenDrive geometry)
    """
    pointSet = etree.Element("pointSet")
    pointList = []
    sCoordinates = []

    GA = to_dict(geometry)
    s = 0.0
    p = line_interp(GA, 0)
    lineIncrement = line_interp(GA, discretizeDistance) - p
    lastS = GA["length"] - sectionEndOffset

    while s < lastS:
        pwgs84 = pyproj.transform(
            xodrproj, apolloproj, p[0], p[1])  # convert to wgs84
        # apollo wants long/lat, flip coordinates
        pwgs84 = np.flip(pwgs84).tolist()
        pointSet.append(point(*pwgs84))
        pointList.append(np.copy(p))
        sCoordinates.append(s)
        s += discretizeDistance
        p += lineIncrement

    endPoint = line_interp(GA, lastS)
    endPointwgs84 = pyproj.transform(
        xodrproj, apolloproj, endPoint[0], endPoint[1])
    # apollo wants long/lat, flip coordinates
    endPointwgs84 = np.flip(endPointwgs84).tolist()
    pointSet.append(point(*endPointwgs84))
    pointList.append(endPoint)
    sCoordinates.append(lastS + sectionEndOffset)

    return pointSet, np.array(pointList), np.array(sCoordinates)


def geometry_to_pointSet(geometry, discretizeDistance, laneOffset, xodrproj, apolloproj, sectionEndOffset):
    """placeholder for generating reference line pointSet with geometry node (OpenDRIVE) according to geometry type

    Arguments:
        geometry {etree._Element} -- straight line geometry node of OpenDRIVE
        discretizeDistance {float} -- distance to discretized line
        laneOffset {etree._Element} -- a lateral shift of the lane reference line

    Raises:
        TypeError: not supported geometry type

    Returns:
        pointSet {etree._Element} -- pointSet node of the discretized reference line
        pointList {numpy.array (n,2)} -- discretized reference line
    """
    geoType = geometry[0].tag
    if geoType == "line":
        pointSet, pointList, sCoordinates = line_to_points(
            geometry, discretizeDistance, xodrproj, apolloproj, sectionEndOffset)
    elif geoType == "arc":
        pointSet, pointList, sCoordinates = arc_to_points(
            geometry, discretizeDistance, xodrproj, apolloproj, sectionEndOffset)
    elif geoType == "spiral":
        pointSet, pointList, sCoordinates = spiral_to_points(
            geometry, discretizeDistance, xodrproj, apolloproj, sectionEndOffset)
    else:
        raise TypeError("Geometry type not support: {}".format(geoType))

    # check if laneOffset is empty or zero
    if len(laneOffset) > 1:
        pointSet, pointList, _, _ = points_to_boundary_pointSet(
            geoType, pointList, laneOffset, sCoordinates, 1, xodrproj, apolloproj)
    elif len(laneOffset) == 1:
        offsetDict = to_dict(laneOffset[0])

        # if laneOffset is not zero
        if sum(np.abs(list(map(offsetDict.get, ["a", "b", "c", "d"])))) > 0:
            # direction depend on laneOffset when sign=1
            pointSet, pointList, _, _ = points_to_boundary_pointSet(
                geoType, pointList, laneOffset, sCoordinates, 1, xodrproj, apolloproj)

    return pointSet, pointList, sCoordinates

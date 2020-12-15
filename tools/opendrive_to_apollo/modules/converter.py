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

from adaptor.common_utils import *
from adaptor.geometry_utils import geometry_to_pointSet, points_to_boundary_pointSet
from visualization_tools.map_visualization import plot_map, plot_junction

from adaptor.geometry_parser import geometry_parser
from adaptor.lane_parser import lane_parser, getLaneNeighbors
from adaptor.road_parser import road_parser
from adaptor.laneSection_parser import laneSection_parser

import numpy as np
import argparse
from lxml import etree
from copy import copy
import matplotlib.pyplot as plt
from collections import defaultdict
from tqdm import tqdm
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from shapely.geometry import LineString
#import geopandas
import pyproj

import matplotlib
matplotlib.use('TkAgg')

# NOTE: variables named with _ in the front corresponse to HDMap
# NOTE: run source scripts/apollo_base.sh before executing map generator


class opendrive2apollo_converter:
    def __init__(self, discret=2, plot=False, debug=False, dropShortSegmentLength=0.25, sectionEndOffset=0.0, reverseOrderString=""):
        """initialize class

        Keyword Arguments:
            discret {float} -- distance of discretizing line and curve (default: {2})
            plot {bool}   -- visualize discretized map (default: {False})
            debug {bool}  -- ignore necessary parameters of projecting points in Apollo map generator (default: {False})
            dropShortSegmentLength {float} -- minimum length a road segment shall have, otherwise we ignore it
        """
        self.discret = discret
        self.plot = plot
        self.debug = debug
        self.dropShortSegmentLength = dropShortSegmentLength
        self.sectionEndOffset = sectionEndOffset

        self.root = None
        self._root = etree.Element("OpenDRIVE")  # root of the HDMap

        self.xodrproj = []
        self.apolloproj = []

        # a huge bounding box around Munich
        self.coordinate_bounding_box = {
            "north": 48.3, "east": 11.8, "south": 48.0, "west": 11.3}

        if self.plot:
            fig = plt.figure()
            plt.axis("equal")
            self.ax = fig.gca()

            #world = geopandas.read_file(geopandas.datasets.get_path('naturalearth_lowres'))
            # self.ax = world[world.continent == 'Europe'].plot(
            #    color='white', edgecolor='black')
            #self.ax.set_xlim([self.coordinate_bounding_box["west"], self.coordinate_bounding_box["east"]])
            #self.ax.set_ylim([self.coordinate_bounding_box["south"], self.coordinate_bounding_box["north"]])
        else:
            self.ax = []

        if bool(reverseOrderString):
            self.reverseOrderList = reverseOrderString.split(" ")
        else:
            self.reverseOrderList = []

    def read_opendrive_map(self, map):
        """load opendive map into etree

        Arguments:
            map {str} -- path to the OpenDRIVE map

        Raises:
            TypeError: Not a valid OpenDRIVE map
            TypeError: Error during parsing OpenDRIVE map
        """
        try:
            xodrFile = open(map, 'rb')
            parser = etree.XMLParser(remove_blank_text=True, recover=True,
                                     strip_cdata=False, huge_tree=True)
            xodr = etree.parse(xodrFile, parser)
            xodrFile.close()

            self.root = xodr.getroot()
            if self.root.tag != "OpenDRIVE":
                raise TypeError("Not a valid OpenDRIVE map")
        except:
            raise TypeError("Error during parsing OpenDRIVE map")

    def write(self, output):
        """write the result Apollo HDMap to file

        Arguments:
            output {str} -- path to output the HDMap
        """
        xodr = etree.ElementTree(self._root)
        xodr.write(output, pretty_print=True,
                   xml_declaration=True, encoding="utf-8")

    def run(self):
        """convert OpenDRIVE map to HDMap
        """
        self.parse_header(self.root, self._root)
        self.parse_roads(self.root, self._root)
        self.parse_junction(self.root, self._root)
        # self.postprocessLinks(self._root)

    def plot_map(self):
        """plot discretized map with matplotlib
        """
        plt.show()

    def parse_road_lines(self, road):
        """parse pointSet of reference lines (road), boundaries (laneSection), centerLine (lane) and borders (lane)

        Arguments:
            road {etree._Element} -- road element from OpenDRIVE map

        Returns:
            lines {dict} -- discretized lines:
                            routeView          -- reference line
                            centerLineGeoDicts -- centerLine of each lane (except center laneSection)
                            boundariesList     -- boundaries of each laneSection
                            borderGeoDicts     -- border of each lane
                            borderWidthsDicts   -- border width of each border
        """
        # print("road:", road.get("id"))
        lanes = road.find("lanes")
        planView = road.find("planView")
        routeView = etree.Element("routeView")

        # each entry in the list associates with a OpenDRIVE geometry
        centerLineGeoDicts = []
        borderGeoDicts = []
        borderWidthsDicts = []
        boundariesList = []
        discretStepsList = []

        # TODO: looping multiple laneSections is quite ugly
        # key of the defaultdicts coorresponds to laneIdx
        # each entry in the list of defaultdict associates with a laneSection
        centerLineGeoDicts.append(defaultdict(list))
        borderGeoDicts.append(defaultdict(list))
        borderWidthsDicts.append(defaultdict(list))
        boundariesList.append(etree.Element("boundaries"))
        discretStepsList.append([])

        laneSectionIter = iter(lanes.findall("laneSection"))
        laneSection = next(laneSectionIter)  # first laneSection

        laneOffsets = lanes.findall("laneOffset")

        routeViewRefLineList = []

        """
        parse reference line
        """

        for geometry in planView.iterfind("geometry"):
            geoType = geometry[0].tag
            sOffset = float(geometry.get("s"))
            length = float(geometry.get("length"))

            # throw away too short segments
            if length < self.dropShortSegmentLength:
                continue

            # NOTE: here assume the s coordinate of each laneSection
            #       intersects with one of the s coordinate geometry
            while laneSectionIter.has_next() \
                    and round(sOffset, 7) > round(float(laneSection.get("s")), 7):
                laneSection = next(laneSectionIter)

                centerLineGeoDicts.append(defaultdict(list))
                borderGeoDicts.append(defaultdict(list))
                borderWidthsDicts.append(defaultdict(list))
                boundariesList.append(etree.Element("boundaries"))
                discretStepsList.append([])

            pointSet, refLinePointList, discretSteps = geometry_to_pointSet(
                geometry, self.discret, laneOffsets, self.xodrproj, self.apolloproj, self.sectionEndOffset)

            routeViewRefLineList.append(refLinePointList)

            RefLineGeometry = geometry_parser(
                geometry, pointSet, self.xodrproj, self.apolloproj)
            routeView.append(copy(RefLineGeometry))

            discretSteps += sOffset
            discretStepsList[-1].append(discretSteps)

            """
            parse borders of lane, depends on RefLineGeometry and refLinePointList
            """
            left = laneSection.find("left")
            right = laneSection.find("right")
            bordersPointLists = dict()

            if left is not None:
                leftLanesDict = {int(lane.get("id")): lane for lane in left}
                sortedLeftLanesDictKeys = sorted(leftLanesDict.keys())
                pointList = refLinePointList

                for laneIdx in sortedLeftLanesDictKeys:
                    widths = leftLanesDict[laneIdx].findall(
                        "width")

                    borderPointSet, pointList, borderLength, borderWidths = points_to_boundary_pointSet(
                        geoType, pointList, widths, discretSteps, 1, self.xodrproj, self.apolloproj)

                    bordersPointLists[laneIdx] = pointList
                    borderWidthsDicts[-1][laneIdx].append(borderWidths)

                    borderGeometry = geometry_parser(
                        sOffset, borderLength, borderPointSet, self.xodrproj, self.apolloproj)
                    borderGeoDicts[-1][laneIdx].append(borderGeometry)

            if right is not None:
                rightLanesDict = {int(lane.get("id")): lane for lane in right}
                sortedRightLanesDictKeys = sorted(
                    rightLanesDict.keys(), reverse=True)

                pointList = refLinePointList

                for laneIdx in sortedRightLanesDictKeys:
                    widths = rightLanesDict[laneIdx].findall(
                        "width")

                    borderPointSet, pointList, borderLength, borderWidths = points_to_boundary_pointSet(
                        geoType, pointList, widths, discretSteps, -1, self.xodrproj, self.apolloproj)

                    bordersPointLists[laneIdx] = pointList
                    borderWidthsDicts[-1][laneIdx].append(borderWidths)

                    borderGeometry = geometry_parser(
                        sOffset, borderLength, borderPointSet, self.xodrproj, self.apolloproj)
                    borderGeoDicts[-1][laneIdx].append(borderGeometry)

            borderGeoDicts[-1][0].append(copy(RefLineGeometry))
            bordersPointLists[0] = refLinePointList

            """
            parse centerLine for left and right laneSection, depends on RefLineGeometry and refLinePointList
            """

            centerLineGeoDicts[-1][0].append(copy(RefLineGeometry))

            if left is not None:
                for laneIdx in sortedLeftLanesDictKeys:
                    widths = leftLanesDict[laneIdx].findall("width")
                    centerLinePointSet,  _, length, _ = points_to_boundary_pointSet(
                        geoType, bordersPointLists[laneIdx-1], widths, discretSteps, 0.5, self.xodrproj, self.apolloproj)

                    centerLineGeometry = geometry_parser(
                        sOffset, length, centerLinePointSet, self.xodrproj, self.apolloproj)

                    centerLineGeoDicts[-1][laneIdx].append(centerLineGeometry)

            if right is not None:
                for laneIdx in sortedRightLanesDictKeys:
                    widths = rightLanesDict[laneIdx].findall("width")
                    centerLinePointSet, _, length, _ = points_to_boundary_pointSet(
                        geoType, bordersPointLists[laneIdx+1], widths, discretSteps, -0.5, self.xodrproj, self.apolloproj)

                    centerLineGeometry = geometry_parser(
                        sOffset, length, centerLinePointSet, self.xodrproj, self.apolloproj)

                    centerLineGeoDicts[-1][laneIdx].append(centerLineGeometry)

            """
            parse boundaries of laneSection, depends on RefLineGeometry and refLinePointList

            NOTE: Different from geometry in other location,
                  there is only one geometry (no attribute) under boundary in HDMap
            """
            leftBoundary = etree.SubElement(
                boundariesList[-1], "boundary", {"type": "leftBoundary"})
            rightBoundary = etree.SubElement(
                boundariesList[-1], "boundary", {"type": "rightBoundary"})

            leftBoundaryGeometry = etree.SubElement(leftBoundary, "geometry")
            if left is not None:
                leftMostLaneIdx = max(sortedLeftLanesDictKeys)
                leftBoundaryGeometry.append(
                    copy(borderGeoDicts[-1][leftMostLaneIdx][-1].find("pointSet")))
            else:
                # if no left laneSection, reference line will be the left boundary
                leftBoundaryGeometry.append(copy(RefLineGeometry[0]))

            rightBoundaryGeometry = etree.SubElement(
                rightBoundary, "geometry")
            if right is not None:
                rightMostLaneIdx = min(sortedRightLanesDictKeys)
                rightBoundaryGeometry.append(
                    copy(borderGeoDicts[-1][rightMostLaneIdx][-1].find("pointSet")))
            else:
                # if no right laneSection, reference line will be the right boundary
                rightBoundaryGeometry.append(copy(RefLineGeometry[0]))

        # close gaps: force last point of previous segment to match first point of next segment.
        for idxGeo in range(len(routeViewRefLineList)-1):
            routeViewRefLineList[idxGeo][-1] = routeViewRefLineList[idxGeo+1][0]
            routeView.findall("geometry")[idxGeo].find("pointSet").findall("point")[-1].set("x",
                                                                                            routeView.findall("geometry")[idxGeo+1].find("pointSet").findall("point")[0].get("x"))
            routeView.findall("geometry")[idxGeo].find("pointSet").findall("point")[-1].set("y",
                                                                                            routeView.findall("geometry")[idxGeo+1].find("pointSet").findall("point")[0].get("y"))
            for _, thisitem in centerLineGeoDicts[0].items():
                thisitem[idxGeo].find("pointSet").findall("point")[-1].set("x",
                                                                           thisitem[idxGeo+1].find("pointSet").findall("point")[0].get("x"))
                thisitem[idxGeo].find("pointSet").findall("point")[-1].set("y",
                                                                           thisitem[idxGeo+1].find("pointSet").findall("point")[0].get("y"))
            # left
            boundariesList[0].findall("boundary")[idxGeo*2].find("geometry").find("pointSet")[-1].set("x",
                                                                                                      boundariesList[0].findall("boundary")[(idxGeo+1)*2].find("geometry").find("pointSet")[0].get("x"))
            boundariesList[0].findall("boundary")[idxGeo*2].find("geometry").find("pointSet")[-1].set("y",
                                                                                                      boundariesList[0].findall("boundary")[(idxGeo+1)*2].find("geometry").find("pointSet")[0].get("y"))
            # right
            boundariesList[0].findall("boundary")[idxGeo*2+1].find("geometry").find("pointSet")[-1].set("x",
                                                                                                        boundariesList[0].findall("boundary")[(idxGeo+1)*2+1].find("geometry").find("pointSet")[0].get("x"))
            boundariesList[0].findall("boundary")[idxGeo*2+1].find("geometry").find("pointSet")[-1].set("y",
                                                                                                        boundariesList[0].findall("boundary")[(idxGeo+1)*2+1].find("geometry").find("pointSet")[0].get("y"))
            for _, thisitem in borderGeoDicts[0].items():
                thisitem[idxGeo].find("pointSet")[-1].set("x",
                                                          thisitem[idxGeo+1].find("pointSet")[0].get("x"))
                thisitem[idxGeo].find("pointSet")[-1].set("y",
                                                          thisitem[idxGeo+1].find("pointSet")[0].get("y"))
            for _, thisitem in borderWidthsDicts[0].items():
                thisitem[idxGeo][-1] = thisitem[idxGeo+1][0]

        lines = {}
        lines["routeView"] = routeView
        lines["centerLineGeoDicts"] = centerLineGeoDicts
        lines["boundariesList"] = boundariesList
        lines["borderGeoDicts"] = borderGeoDicts
        lines["borderWidthsDicts"] = borderWidthsDicts
        lines["discretStepsList"] = discretStepsList

        return lines

    def parse_roads(self, root, _root):
        """parse roads from OpenDRIVE map to HDMap

        Arguments:
            root {etree._Element} -- root node of OpenDRIVE
            _root {etree._Element} -- root node of HDMap
        """
        processBar = tqdm(root.findall("road"),
                          desc="Converting %s" % root.find(
                              "header").get("name"),
                          unit="road")
        for road in processBar:
            lines = self.parse_road_lines(road)

            if self.plot:
                plot_map(self.ax, lines)

            link = road.find("link")
            road_id = road.get("id")
            lanes = self.parse_lanes(road.find("lanes"), lines, road_id, link)

            _road = road_parser(road, link, lines["routeView"], lanes)
            _root.append(_road)

    def parse_lanes(self, lanes, lines, roadId, roadLink):
        """parse lanes from OpenDRIVE map to HDMap

        Arguments:
            lanes {etree._Element} -- lanes node of OpenDRIVE
            lines {dict} -- as descripted in parse_road_lines
            roadId {str} -- id of the road these lanes belongs to
            roadLink {etree._Element} -- road link element of the road this lane belongs to 

        Returns:
            _lanes {etree._Element} -- lanes node of HDMap
        """
        _lanes = etree.Element("lanes")

        # for each OpenDRIVE laneSection
        # NOTE: here assumes coordinate of geometry intersects with the one of the laneSections
        for laneSection, centerLineGeoDict, boundaries, borderGeoDict, borderWidthDict, discretSteps in \
                zip(lanes.findall("laneSection"), lines["centerLineGeoDicts"], lines["boundariesList"],
                    lines["borderGeoDicts"], lines["borderWidthsDicts"], lines["discretStepsList"]):
            _laneSection = laneSection_parser(laneSection, boundaries)
            _lanes.append(_laneSection)

            """
            parse attributes and children of left, center, right laneSection
            """

            for section, laneSectionString in zip(map(laneSection.find, laneSectionStrings), laneSectionStrings):
                if section is not None:
                    lanePosition = etree.SubElement(
                        _laneSection, laneSectionString)

                    for lane in section:
                        _lane = self.parse_lane(
                            lane, centerLineGeoDict, borderGeoDict, borderWidthDict, discretSteps, roadId, roadLink)
                        lanePosition.append(_lane)

        return _lanes

    def parse_lane(self, lane, centerLineGeoDict, borderGeoDict, borderWidthsDict, discretStepsList, roadIdx, roadLink):
        """parse lane from OpenDRIVE map to HDMap

        Arguments:
            lane {etree._Element} -- lane node of OpenDRIVE
            centerLineGeoDict {dict} -- as descripted in parse_road_lines (geometry(s) coorrespond to a lanSection)
            borderGeoDict {dict} -- as descripted in parse_road_lines (geometry(s) coorrespond to a lanSection)
            borderWidthsDict {dict} -- as descripted in parse_road_lines (widthsDict(s) coorrespond to a lanSection)
            roadIdx {str} -- road id this lane belongs to
            roadLink {etree._Element} -- road link element of the road this lane belongs to 

        Returns:
            _lane {etree._Element} -- lane node of HDMap
        """
        laneIdx = int(lane.get("id"))
        link = lane.find("link")
        speed = lane.find("speed")
        _lane = lane_parser(laneIdx, generate_lane_uid(roadIdx, laneIdx),
                            lane.get("type"), "bidirection", "noTurn", link, speed, roadLink)

        neighborLinks = getLaneNeighbors(lane, roadIdx)
        if neighborLinks:
            links = _lane.find("link")
            if links is None:
                links = etree.Element("link")
                _lane.append(links)
            for nl in neighborLinks:
                links.append(nl)

        reverseOrder = False
        geoDictIteratorDirection = 1
        # if generate_lane_uid(roadIdx, laneIdx) in self.reverseOrderList: # TODO remove
        # All right lanes are oriented in the opposite direction.
        if float(laneIdx) > 0:
            reverseOrder = True
            geoDictIteratorDirection = -1

        border = _lane.find("border")
        centerLine = _lane.find("centerLine")
        sampleAssociates = _lane.find("sampleAssociates")
        roadSampleAssociations = _lane.find("roadSampleAssociations")

        lastDiscretStep = None
        for i, (centerLineGeo, borderGeo, discretSteps) in \
                enumerate(zip(centerLineGeoDict[laneIdx][::geoDictIteratorDirection], borderGeoDict[laneIdx][::geoDictIteratorDirection], discretStepsList[::geoDictIteratorDirection])):
            if laneIdx != 0:
                if reverseOrder:
                    pts = centerLineGeo.find("pointSet").findall("point")[::-1]
                    ps = centerLineGeo.find("pointSet")
                    centerLineGeo.remove(ps)
                    ps = etree.SubElement(centerLineGeo, "pointSet")
                    for p in pts:
                        ps.append(p)
                centerLine.append(centerLineGeo)
                # TODO do I have to rewind the order back again?

            if reverseOrder:
                pts = borderGeo.find("pointSet").findall("point")[::-1]
                ps = borderGeo.find("pointSet")
                borderGeo.remove(ps)
                ps = etree.SubElement(borderGeo, "pointSet")
                for p in pts:
                    ps.append(p)
            border.append(borderGeo)
            # TODO do I have to rewind the order back again?

            # parse border types. NOTE: Apollo's documentation is wrong here (in 5.5)! <borderType> is child of <border>,
            # the tag <borderTypes> does not exist in the code.
            # TODO: parse with multiple roadMark (we dont have that atm)
            roadMark = lane.find("roadMark")
            borderType = etree.SubElement(border, "borderType", {
                "sOffset": centerLineGeo.get("sOffset"), "eOffset": centerLineGeo.get("length")})

            type_ = roadMark.get("type") if roadMark is not None else None
            if type_ in xodrBorderTypes:
                borderType.set("type", type_)
            else:
                borderType.set(
                    "type", apolloBorderTypes[type_] if type_ in apolloBorderTypes else "none")

            color = roadMark.get("color") if roadMark is not None else None
            borderType.set(
                "color", color if color in supportedBorderColors else "none")

            # generate sampleAssociates and roadSampleAssociations
            if laneIdx != 0:
                for j, s in enumerate(discretSteps):
                    if s != lastDiscretStep:
                        lastDiscretStep = s
                    else:
                        continue

                    # TODO not so nice... but borderWidthsDict is longer!
                    if reverseOrder:
                        ii = len(discretStepsList) - 1 - i
                    else:
                        ii = i

                    halfLaneWidth = borderWidthsDict[laneIdx][ii][j]/2

                    sampleAssociate = etree.SubElement(
                        sampleAssociates, "sampleAssociate", {"sOffset": to_str(s), "leftWidth": to_str(halfLaneWidth), "rightWidth": to_str(halfLaneWidth)})

                    disToLeftBoundary = sum(
                        [v[ii][j] for k, v in borderWidthsDict.items() if k < laneIdx])+halfLaneWidth
                    disToRightBoundary = sum(
                        [v[ii][j] for k, v in borderWidthsDict.items() if k > laneIdx])+halfLaneWidth
                    roadSampleAssociation = etree.SubElement(roadSampleAssociations, "sampleAssociation", {
                        "sOffset": to_str(s), "leftWidth": to_str(disToLeftBoundary), "rightWidth": to_str(disToRightBoundary)})

        return _lane

    def get_junction_roads(self, root):
        """get all roads connecting to a junction

        Arguments:
            root {etree._Element} -- root node of OpenDRIVE

        Returns:
            roads {dict} -- dict contains all road connecting to a junction
        """
        roads = {}

        for road in root.iterfind("road"):
            junctionIdx = road.get("junction")
            roadIdx = int(road.get("id"))

            if junctionIdx is not None and junctionIdx != "-1":
                roads[roadIdx] = {}
                laneSection = road.find("lanes").find("laneSection")
                for section in map(laneSection.find, laneSectionStrings):
                    if section is not None:
                        for lane in section.iterfind("lane"):
                            roads[roadIdx][int(lane.get("id"))] = lane

        return roads

    def parse_junction(self, root, _root):
        """parse lane from OpenDRIVE map to HDMap

        Arguments:
            root {[type]} -- root node of OpenDRIVE
            _root {[type]} -- root node of HDMap

        Raises:
            ValueError: for debugging
        """
        roadsWithJunction = self.get_junction_roads(_root)

        for junction in root.iterfind("junction"):
            _junction = etree.SubElement(_root, "junction")
            _junction.set("id", junction.get("id"))

            outline = etree.SubElement(_junction, "outline")
            cornersPoint = []

            for connection in junction.iterfind("connection"):

                # this connection is unnecessary for apollo.
                _connection = etree.SubElement(_junction, "connection")
                copy_attributes(connection, _connection, [
                                "id", "incomingRoad", "connectingRoad", "contactPoint"])
                copy_children(connection, _connection)

                # TODO what to do with the contact point?
                incomingRoad_id = _connection.get("incomingRoad")
                connectingRoad_id = _connection.get("connectingRoad")
                lanelink_from_id = []
                lanelink_to_id = []
                for lanelink in _connection.findall("laneLink"):
                    lanelink_from_id.append(lanelink.get("from"))
                    lanelink_to_id.append(lanelink.get("to"))

                # TODO do we also have to add the predecessors here the same way?
                for apollo_road in _root.iterfind("road"):
                    if apollo_road.get("id") == incomingRoad_id:
                        # for apollo, I do not have to fill the road linkage

                        # navigate to lanes -> laneSection -> right OR left -> lane
                        # if lanelink_from_id matches lane id : add link with connecting road
                        for apollo_lanes in apollo_road.iterfind("lanes"):
                            apllo_laneSection = apollo_lanes.find(
                                "laneSection")

                            # not a very nice impl: left and right is a copy
                            apollo_left = apllo_laneSection.find("left")
                            for apollo_left_lane in apollo_left.findall("lane"):
                                for idx, this_lanelink_from_id in enumerate(lanelink_from_id):
                                    if apollo_left_lane.get("id") == this_lanelink_from_id:
                                        apollo_left_lane_link = apollo_left_lane.find(
                                            "link")
                                        if apollo_left_lane_link is None:  # no link present, we have to add a link a child, we have to add <link> first
                                            apollo_left_lane_link = etree.SubElement(
                                                apollo_left_lane, "link")
                                            apollo_left_lane.append(
                                                apollo_left_lane_link)
                                        successor = etree.SubElement(
                                            apollo_left_lane_link, "successor")
                                        successor.set("id", generate_lane_uid(
                                            connectingRoad_id, lanelink_to_id[idx]))
                                        apollo_left_lane_link.append(successor)

                            apollo_right = apllo_laneSection.find("right")
                            for apollo_right_lane in apollo_right.findall("lane"):
                                for idx, this_lanelink_from_id in enumerate(lanelink_from_id):
                                    if apollo_right_lane.get("id") == this_lanelink_from_id:
                                        apollo_right_lane_link = apollo_right_lane.find(
                                            "link")
                                        if apollo_right_lane_link is None:  # no link present, we have to add a link a child, we have to add <link> first
                                            apollo_right_lane_link = etree.SubElement(
                                                apollo_right_lane, "link")
                                            apollo_right_lane.append(
                                                apollo_right_lane_link)
                                        successor = etree.SubElement(
                                            apollo_right_lane_link, "successor")
                                        successor.set("id", generate_lane_uid(
                                            connectingRoad_id, lanelink_to_id[idx]))
                                        apollo_right_lane_link.append(
                                            successor)

                """
                parse outline & junction cornerGlobal
                """
                connectingRoad = roadsWithJunction[int(
                    _connection.get("connectingRoad"))]

                if len(_connection) == 0:
                    # NOTE: lanes store in descending ID
                    minLaneIdx = min(connectingRoad.keys())
                    maxLaneIdx = max(connectingRoad.keys())

                    # NOTE: get cornerPoints with the path:
                    #       road->lane->border->geometry[contactPoint]->pointSet->point[contactPoint]
                    leftFirstCornerPoint = connectingRoad[minLaneIdx].find(
                        "border").findall("geometry")[0].find("pointSet")[0]
                    leftLastCornerPoint = connectingRoad[minLaneIdx].find(
                        "border").findall("geometry")[-1].find("pointSet")[-1]
                    rightFirstCornerPoint = connectingRoad[maxLaneIdx].find(
                        "border").findall("geometry")[0].find("pointSet")[0]
                    rightLastCornerPoint = connectingRoad[maxLaneIdx].find(
                        "border").findall("geometry")[-1].find("pointSet")[-1]

                    cornersPoint.append(leftFirstCornerPoint)
                    cornersPoint.append(leftLastCornerPoint)
                    cornersPoint.append(rightFirstCornerPoint)
                    cornersPoint.append(rightLastCornerPoint)
                else:
                    for laneLink in _connection:
                        toLaneIdx = int(laneLink.get("to"))
                        if toLaneIdx < 0:
                            leftFirstCornerPoint = connectingRoad[toLaneIdx].find(
                                "border").findall("geometry")[0].find("pointSet")[0]
                            leftLastCornerPoint = connectingRoad[toLaneIdx].find(
                                "border").findall("geometry")[-1].find("pointSet")[-1]
                            rightFirstCornerPoint = connectingRoad[toLaneIdx+1].find(
                                "border").findall("geometry")[0].find("pointSet")[0]
                            rightLastCornerPoint = connectingRoad[toLaneIdx+1].find(
                                "border").findall("geometry")[-1].find("pointSet")[-1]
                        elif toLaneIdx > 0:
                            leftFirstCornerPoint = connectingRoad[toLaneIdx-1].find(
                                "border").findall("geometry")[0].find("pointSet")[0]
                            leftLastCornerPoint = connectingRoad[toLaneIdx-1].find(
                                "border").findall("geometry")[-1].find("pointSet")[-1]
                            rightFirstCornerPoint = connectingRoad[toLaneIdx].find(
                                "border").findall("geometry")[0].find("pointSet")[0]
                            rightLastCornerPoint = connectingRoad[toLaneIdx].find(
                                "border").findall("geometry")[-1].find("pointSet")[-1]
                        # in junction, lane connection of center laneSection is ignored

                        cornersPoint.append(leftFirstCornerPoint)
                        cornersPoint.append(leftLastCornerPoint)
                        cornersPoint.append(rightFirstCornerPoint)
                        cornersPoint.append(rightLastCornerPoint)

            # Use the convex hull of all corner points as global corners
            rawpts = np.zeros(shape=(0, 2))
            for p in cornersPoint:
                rawpts = np.append(rawpts, [[p.get("x"), p.get("y")]], axis=0)
            hull = ConvexHull(rawpts)
            #cornerpoints_hull_xy = rawpts[hull.vertices,:]

            convhull_cornersPoint = [cornersPoint[i] for i in hull.vertices]

            for p in convhull_cornersPoint:
                cornerCoord = {string: p.get(
                    string) for string in coordStrings}
                cornerGlobal = etree.SubElement(
                    outline, "cornerGlobal", cornerCoord)

            if self.plot:
                plot_junction(self.ax, convhull_cornersPoint)

    def parse_header(self, root, _root):
        """parse header from OpenDRIVE map to HDMap
        check information needed in Apollo map generator
        modify the georef string
        parse and store the map origin

        Arguments:
            root {[type]} -- root node of OpenDRIVE
            _root {[type]} -- root node of HDMap

        Raises:
            ValueError: see comment
        """

        # get header
        header = root.find("header")

        # set fortiss as vendor in header tag
        header.set("vendor", "fortiss")

        # set north east south west bounding box
        header.set("north", to_str(self.coordinate_bounding_box["north"]))
        header.set("south", to_str(self.coordinate_bounding_box["south"]))
        header.set("east", to_str(self.coordinate_bounding_box["east"]))
        header.set("west", to_str(self.coordinate_bounding_box["west"]))

        # get geo reference positions
        georef_element = header.find("geoReference")
        if georef_element is None:
            raise ValueError(
                "No geoReference provided in file header. We need this to convert the points to WGS84!")
        # probably this can also by done by a geodesy library.
        georef = georef_element.text.split("+")
        origin_long = None
        origin_lat = None
        for entry in georef:
            if not entry.find("lon") == -1:
                origin_long = float(entry.split("=")[1])
            if not entry.find("lat") == -1:
                origin_lat = float(entry.split("=")[1])
        if origin_lat == None or origin_long == None:
            raise ValueError(
                "geoReference has to provide long and lat values. We need this to convert the points to WGS84!")

        # # modify georef string
        # # here we set the projection to latlong (otherwise the wgs84 values are not correctly transformed to utm values by apollo)
        # georef_replace_string = "+proj=latlong +lat_0=" + to_str(self.origin_lat) + " +lon_0=" + to_str(self.origin_long) + " +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs "
        # header.remove(header.find("geoReference"))
        # geoReference_element = etree.SubElement(header, "geoReference")
        # geoReference_element.text = etree.CDATA(georef_replace_string)

        self.xodrproj = pyproj.Proj(header.find("geoReference").text)
        self.apolloproj = pyproj.Proj('epsg:4326')  # wgs84
        # self.apolloproj = pyproj.Proj(proj="utm", zone="32", ellps="WGS84", datum="WGS84") # debugging: utm coordinates
        # self.apolloproj = self.xodrproj # debugging: no trafo

        header.remove(header.find("geoReference"))
        geoReference_element = etree.SubElement(header, "geoReference")
        geoReference_element.text = etree.CDATA(self.apolloproj.srs)

        # check if header data is consistent
        if not self.debug:
            try:
                east = float(header.get("east"))
                west = float(header.get("west"))

                if east == west:
                    raise ValueError(
                        "East and west cannot be the same for projecting points")

                eastZone = get_long_zone(east)
                westZone = get_long_zone(west)

                if eastZone != westZone:
                    # Apollo checks longitude zone of east (max longitude) and west (min longitude), they must be in the same zone
                    raise ValueError(
                        "Map in more than one longitude zone is not supported by Apollo")

                if header.find("geoReference") is None:
                    # add empty geoReference
                    # _geoReference = etree.SubElement(header, "geoReference")
                    # _geoReference.text = etree.CDATA("+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs>")
                    raise ValueError(
                        "GeoReference is needed to project points in Apollo proto map generator")
            except Exception as e:
                print("Error:\t", e)
                print("\nAdd --debug to bypass all the checkings")
                raise SystemExit

        # write header
        _root.append(copy(header))


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="A tool for converting standard OpenDRIVE map to Apollo HDMAP OpenDRIVE format.",
        prog="parser.py")

    parser.add_argument("-m", "--map", metavar="file_location", action="store", type=str, required=True,
                        help="Specify the input location of standard OpenDRIVE map.")

    parser.add_argument("-o", "--output", metavar="file_location", action="store", type=str, required=False,
                        help="Specify the output location of HDMAP OpenDRIVE map.")

    parser.add_argument("-d", "--discretization", metavar="distance", action="store", type=float, required=False, default=1,
                        help="Specify distance to discretization the map into points.")

    parser.add_argument("-p", "--plot", action="store_const", const=True,
                        help="Plot the output HDMAP OpenDRIVE map.")

    parser.add_argument("--debug", action="store_const", const=True,
                        help="Run in debugging mode, ignore all checks.")

    parser.add_argument("-s", "--dropShortSegmentLength", metavar="distance", action="store", type=float, required=False, default=0.25,
                        help="Minimum length a road segment shall have (otherwise it is ignored).")

# TODO do not use atm. it avoids path intersections but somehow messes up the length or s coordinates!!
#    parser.add_argument("-e", "--sectionEndOffset", metavar="distance", action="store", type=float, required=False, default=0.4,
#                        help="geometry sections are not sampled until the total length but until length minus this offset to avoid intersecting sections.")

# TODO unused atm. seems more elegant to look for lane ids > 0 to switch the direction
#    parser.add_argument("-r", "--reverseOrderString", metavar="", action="store", type=str, required=False, default="",
#                        help="Segments to reverse")

    args = parser.parse_args()

    converter = opendrive2apollo_converter(
        discret=args.discretization,
        plot=args.plot,
        debug=args.debug,
        dropShortSegmentLength=args.dropShortSegmentLength,
        sectionEndOffset=0.0,
        reverseOrderString="")
    converter.read_opendrive_map(args.map)
    converter.run()

    if args.output is not None:
        converter.write(args.output)

    if args.plot:
        converter.plot_map()

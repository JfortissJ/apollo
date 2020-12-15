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

import argparse
from networkx.drawing.nx_agraph import to_agraph
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
from collections import defaultdict
from lxml import etree
from modules.adaptor.common_utils import *
import sys
sys.path.append("./")


def parse_lanes_node(G, root, roadContactLane, roadLaneSectionLanes, nodeLayout):
    for road in root.iterfind("road"):
        roadId = int(road.get("id"))
        # print(roadId)

        laneSections = road.find("lanes").findall("laneSection")
        for i, laneSection in enumerate(laneSections):
            for section in map(laneSection.find, laneSectionStrings[:2]):
                if section is not None:
                    for lane in section.iterfind("lane"):
                        laneId = int(lane.get("id"))
                        laneUId = lane.get("uid")
                        node = (roadId, laneId, laneUId)
                        # node = (roadId, laneId)
                        roadLaneSectionLanes[roadId][i, laneId] = node

                        pointSet = lane.find("centerLine").find(
                            "geometry").find("pointSet")
                        p = pointSet[len(pointSet)//2]
                        x = float(p.get("x"))
                        y = float(p.get("y"))
                        nodeLayout[node] = [x, y]
                        G.add_node(node)

                        if i == 0:
                            roadContactLane[roadId, laneId, "start"] = node
                        if i == len(laneSections)-1:
                            roadContactLane[roadId, laneId, "end"] = node


def parse_lanes_edge(G, root, roadContactLane, roadLaneSectionLanes, roadJunctionContactPoint):
    for road in root.iterfind("road"):

        roadId = int(road.get("id"))
        roadLink = road.find("link")

        if roadLink is not None:
            roadPredecessor = roadLink.find("predecessor")
            roadSuccessor = roadLink.find("successor")

            roadPredecessorId = None
            if roadPredecessor is not None:
                if roadPredecessor.get("elementType") == "road":
                    roadPredecessorId = int(roadPredecessor.get("elementId"))
                    roadPredecessorContactPoint = roadPredecessor.get(
                        "contactPoint")
                elif roadPredecessor.get("elementType") == "junction":
                    elementId = int(roadPredecessor.get("elementId"))
                    roadJunctionContactPoint[roadId, elementId] = "start"

            roadSuccessorId = None
            if roadSuccessor is not None:
                if roadSuccessor.get("elementType") == "road":
                    roadSuccessorId = int(roadSuccessor.get("elementId"))
                    roadSuccessorContactPoint = roadSuccessor.get(
                        "contactPoint")
                elif roadSuccessor.get("elementType") == "junction":
                    elementId = int(roadSuccessor.get("elementId"))
                    roadJunctionContactPoint[roadId, elementId] = "end"
        else:
            roadPredecessor = None
            roadSuccessor = None

        laneSections = road.find("lanes").findall("laneSection")
        for i, laneSection in enumerate(laneSections):
            for section in map(laneSection.find, laneSectionStrings[:2]):
                if section is not None:
                    for lane in section.iterfind("lane"):
                        laneId = int(lane.get("id"))
                        laneUId = lane.get("uid")
                        node = (roadId, laneId, laneUId)
                        # node = (roadId, laneId)

                        laneLink = lane.find("link")

                        if laneLink is not None:
                            lanePredecessor = laneLink.find("predecessor")
                            laneSuccessor = laneLink.find("successor")

                            lanePredecessorId = int(lanePredecessor.get(
                                "id")) if lanePredecessor is not None else None
                            laneSuccessorId = int(laneSuccessor.get(
                                "id")) if laneSuccessor is not None else None
                        else:
                            lanePredecessorId = None
                            laneSuccessorId = None

                        if i in {0, len(laneSections)-1}:
                            if i == 0 and roadPredecessorId is not None and lanePredecessorId is not None:
                                G.add_edge(node, roadContactLane[
                                    roadPredecessorId, lanePredecessorId, roadPredecessorContactPoint])
                            # elif (roadPredecessorId is None) != (lanePredecessorId is None):
                            #     print(
                            #         "WARNING: At road {}, lane {}, predecessor information must be presented at 'roadLink' and 'link' under 'lane'".format(roadId, laneId))
                            if i == len(laneSections)-1 and roadSuccessorId is not None and laneSuccessorId is not None:
                                G.add_edge(node, roadContactLane[
                                    roadSuccessorId, laneSuccessorId, roadSuccessorContactPoint])
                            # elif (roadSuccessorId is None) != (laneSuccessorId is None):
                            #     print(
                            #         "WARNING: At road {}, lane {}, Successor information must be presented at 'roadLink' and 'link' under 'lane'".format(roadId, laneId))
                        else:
                            if laneLink is not None:
                                if lanePredecessor is not None:
                                    G.add_edge(
                                        node, roadLaneSectionLanes[roadId][i-1, lanePredecessorId])
                                if laneSuccessor is not None:
                                    G.add_edge(
                                        node, roadLaneSectionLanes[roadId][i+1, laneSuccessorId])


def parse_junction_edge(G, root, roadContactLane, roadLaneSectionLanes, roadJunctionContactPoint):
    for junction in root.iterfind("junction"):
        for connection in junction.iterfind("connection"):
            incomingRoadId = int(connection.get("incomingRoad"))
            connectingRoadId = int(connection.get("connectingRoad"))
            contactPoint = connection.get("contactPoint")

            for laneLink in connection.iterfind("laneLink"):
                fromId = int(laneLink.get("from"))
                toId = int(laneLink.get("to"))

                fromNode = roadContactLane[incomingRoadId, fromId, "start"]
                toNode = roadContactLane[connectingRoadId, toId, contactPoint]
                G.add_edge(fromNode, toNode)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="A tool for visualizing map connectivities on lane level.",
        prog="connection_visualization.py")

    parser.add_argument("-m", "--map", metavar="file_location", action="store", type=str, required=True,
                        help="Specify the input location of Apollo HPMap")

    parser.add_argument("-o", "--output", metavar="file_location", action="store", type=str, required=False,
                        help="Specify the output location of pdf in dot format")

    args = parser.parse_args()

    try:
        xodrFile = open(args.map, 'rb')
        parser = etree.XMLParser(remove_blank_text=True,
                                 strip_cdata=False, huge_tree=True)
        xodr = etree.parse(xodrFile, parser)
        root = xodr.getroot()

        if root.tag != "OpenDRIVE":
            raise ValueError()
    except:
        raise TypeError("Not a valid XODR format")

    G = nx.DiGraph()

    roadContactLane = dict()
    roadLaneSectionLanes = defaultdict(dict)
    roadJunctionContactPoint = dict()
    nodeLayout = dict()
    parse_lanes_node(G, root, roadContactLane,
                     roadLaneSectionLanes, nodeLayout)
    parse_lanes_edge(G, root, roadContactLane,
                     roadLaneSectionLanes, roadJunctionContactPoint)
    parse_junction_edge(G, root, roadContactLane,
                        roadLaneSectionLanes, roadJunctionContactPoint)

    if args.output:
        A = to_agraph(G)
        A.draw(args.output, format="pdf", prog="dot")

    nx.draw(G, with_labels=True, pos=nodeLayout,
            connectionstyle='arc3, rad=0.1')
    plt.show()

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

from adaptor.common_utils import supportedLaneTypes, generate_lane_uid, to_str

from lxml import etree
from copy import copy


def lane_parser(id, uid, laneType, direction, turnType, link, speed, road_link):
    """create lane ndoe (HDMap)

    Arguments:
        id {str}
        uid {str}
        laneType {str}
        direction {str}
        turnType {str}
        link {etree._Element} lane link element, contains lane ids
        speed {etree._Element}
        road_link {etree._Element} road link element of the road this lane belongs to 

    Returns:
        lane {etree._Element} -- parsed lane node
    """
    # attributes
    lane = etree.Element("lane")
    lane.set("id", str(id))
    lane.set("uid", str(uid))
    lane.set("type", laneType if laneType in supportedLaneTypes else "none")
    lane.set("direction", direction)  # not sure what it is
    lane.set("turnType", turnType)  # not sure what it is

    # We reversed the direction of the right lanes (with id > 0).
    # Therefore successors and predecessors are switched
    switch_links = False
    if id > 0:
        switch_links = True

    # Find successors and predecessors at road (not junction) level
    if link is not None:
        link_road_successor = road_link.find("successor")
        link_road_predecessor = road_link.find("predecessor")
        link_lane_successor = link.find("successor")
        link_lane_predecessor = link.find("predecessor")

        link_elem = etree.Element("link")

        # we not not parse and use contactPoint="start" or "end" here (contactPoints are not used in the apollo code)
        # if link type of the road is "road" -> use id
        if (link_road_successor is not None) and link_road_successor.get("elementType") == "road":
            if (link_lane_successor is not None) and bool(link_road_successor.attrib) and bool(link_lane_successor.attrib):
                if not switch_links:
                    suc_elem = etree.Element("successor")
                    suc_elem.set("id", generate_lane_uid(link_road_successor.get(
                        "elementId"), link_lane_successor.get("id")))
                    link_elem.append(suc_elem)
                else:
                    pre_elem = etree.Element("predecessor")
                    pre_elem.set("id", generate_lane_uid(link_road_successor.get(
                        "elementId"), link_lane_successor.get("id")))
                    link_elem.append(pre_elem)

        if (link_road_predecessor is not None) and link_road_predecessor.get("elementType") == "road":
            if (link_lane_predecessor is not None) and bool(link_road_predecessor.attrib) and bool(link_lane_predecessor.attrib):
                if not switch_links:
                    pre_elem = etree.Element("predecessor")
                    pre_elem.set("id", generate_lane_uid(link_road_predecessor.get(
                        "elementId"), link_lane_predecessor.get("id")))
                    link_elem.append(pre_elem)
                else:
                    suc_elem = etree.Element("successor")
                    suc_elem.set("id", generate_lane_uid(link_road_predecessor.get(
                        "elementId"), link_lane_predecessor.get("id")))
                    link_elem.append(suc_elem)
        # if link type of the road is "junction" -> find the correct road from the junction element, see junction parser

        # we found any connection
        if len(link_elem):
            lane.append(link_elem)

    if id != 0:
        etree.SubElement(lane, "centerLine")
    etree.SubElement(lane, "border")

    if id != 0 and speed is not None:
        # TODO: value can be converted according to the "unit" of <speed>
        etree.SubElement("speed", {"min": to_str(0), "max": speed.find("max")})

    # sampleAssociate: distance of the center line of the current lane to the closet left/right border
    etree.SubElement(lane, "sampleAssociates")
    # roadSampleAssociation: distance of the center line of the current lane to the left/right boundary
    etree.SubElement(lane, "roadSampleAssociations")

    return lane


def getLaneNeighbors(lane, roadId):
    """ find the neighboring lanes on a multi-lane road segment from a given lane element

    Arguments:
        lane {etree._Element} apollo map lane element
        roadId {str} road id the lane belongs to 

    Returns:
         [{etree._Element}] -- list of left and right lane neighbor attributes. Empty if none.
    """

    direction = lane.getparent()
    laneId = int(lane.get("id"))
    if direction.tag == "left":
        nextRightId = laneId + 1
        nextLeftId = laneId - 1
        if nextLeftId == 0:
            nextLeftId = nextLeftId - 1
    elif direction.tag == "right":
        nextRightId = laneId - 1
        nextLeftId = laneId + 1
        if nextLeftId == 0:
            nextLeftId = nextLeftId + 1
    else:  # center
        return []

    laneSection = direction.getparent()
    return neighborAttributeHelper(nextRightId, laneId, laneSection, roadId) + \
        neighborAttributeHelper(nextLeftId, laneId, laneSection, roadId)


def neighborAttributeHelper(nextId, laneId, laneSection, roadId):
    """ helper function: find the neighboring lanes on a multi-lane road segment from lane id
    NOTE this code assumes that left lanes have positive ids and right lanes negative ids!

    Arguments:
        nextId {int} neighbor candidate id
        laneId {int} id of the current lane
        laneSection {etree._Element} parent laneSection element of both lanes
        roadId {str} road id the lane belongs to 

    Returns:
        [{etree._Element}] -- list of left or right lane neighbor attributes. Empty if none.
    """

    lanesList = []

    right = laneSection.find("right")
    if nextId < 0 and right is not None:
        lanesList = lanesList + right.findall("lane")

    left = laneSection.find("left")
    if nextId > 0 and left is not None:
        lanesList = lanesList + left.findall("lane")

    for l in lanesList:
        if nextId == int(l.get("id")):
            return [generateNeighborAttributeHelper(roadId, nextId, laneId)]

    return []


def generateNeighborAttributeHelper(roadId, nextId, laneId):
    """ helper function: generate neighbor tag.
    Here we calculate the right/left and same/opposite attributes from the given ids
    NOTE this code assumes that left lanes have positive ids and right lanes negative ids!

    Arguments:
        nextId {int} -- neighbor candidate id
        laneId {int} -- id of the current lane
        roadId {str} -- road id the lane belongs to 

    Returns:
        neighbor {etree._Element} -- neighbor attribute 
    """

    neighbor = etree.Element("neighbor")
    neighbor.set("id", generate_lane_uid(roadId, nextId))
    if abs(nextId) <= abs(laneId):
        # == : the special case, ids 1 and -1 and direction change
        neighbor.set("side", "left")
    else:
        neighbor.set("side", "right")
    if laneId*nextId > 0:
        neighbor.set("direction", "same")
    else:
        neighbor.set("direction", "opposite")
    return neighbor

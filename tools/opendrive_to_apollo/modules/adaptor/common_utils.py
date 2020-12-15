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

from copy import copy

NUM_FORMAT = "%.16E"

coordStrings = ["x", "y", "z"]
laneSectionStrings = ["left", "right", "center"]
supportedLaneTypes = set(["none", "driving", "biking", "parking", "onRamp",
                          "offRamp", "connectingRamp", "shoulder", "exit"])

xodrBorderTypes = set(["solid", "broken", "curb"])
apolloBorderTypes = {"solid solid": "solid", "solid broken": "broken",
                     "broken solid": "broken", "broken broken": "broken"}  # apollo 5.5 implementation only supports: "solid", "broken", "curb"

# apollo 5.5 implementation only supports white and yellow
supportedBorderColors = set(["white", "yellow"])


def to_str(num):
    """convert a numeric to formated string

    Arguments:
        num {int/float} -- numerics

    Returns:
        str -- formated string
    """
    return NUM_FORMAT % num


def to_dict(element):
    """convert etree._element to dict with float values

    Arguments:
        element {etree._Element}

    Returns:
        dict -- formated dict
    """
    return {k: float(v) for k, v in element.items()}


def copy_children(node, _node, childrenNames=None):
    for child in node:
        if childrenNames is None or child.tag in childrenNames:
            _node.append(copy(child))


def copy_attributes(node, _node, attributeNames=None):
    for key, value in node.attrib.items():
        if attributeNames is None or key in attributeNames:
            _node.set(key, value)


def polynorm(offset, s):
    return offset["a"]+offset["b"]*s+offset["c"]*s**2+offset["d"]*s**3


def generate_lane_uid(road_id, lane_id):
    return str(road_id) + "_" + str(lane_id)


class iter:
    def __init__(self, l):
        self.l = l
        self.i = -1

    def __iter__(self):
        return self

    def has_next(self):
        return not(self.i == len(self.l)-1)

    def get_next(self):
        return self.l[self.i+1] if self.i+1 < len(self.l) else None

    def __next__(self):
        if len(self.l) == 0:
            return None

        if self.i < len(self.l)-1:
            self.i += 1
        return self.l[self.i]


def get_long_zone(longitude):
    if longitude < 0:
        longZone = (180+longitude)/6+1
    else:
        longZone = (longitude/6)+31

    return int(longZone)

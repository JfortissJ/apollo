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

from lxml import etree


def road_parser(road, link, routeView, lanes):
    """create HDMap road node

    Arguments:
        road {etree._Element} -- OpenDRIVE road node
        link {etree._Element} -- OpenDRIVE link node
        routeView {etree._Element} -- parsed routeView node
        lanes {etree._Element} -- parsed routeView node

    Returns:
        _road {etree._Element} -- parsed HDMap road node
    """
    _road = etree.Element("road")
    copy_attributes(road, _road, ["name", "id", "junction"])

    # TODO not needed for apollo?
    if link is not None and len(link) > 0:
        _road.append(link)
    _road.append(routeView)
    _road.append(lanes)

    return _road

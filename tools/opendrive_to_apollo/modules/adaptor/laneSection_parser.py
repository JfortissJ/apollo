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


def laneSection_parser(laneSection, boundaries):
    """create HDMap laneSection node

    Arguments:
        laneSection {etree._Element} -- OpenDRIVE laneSection node
        boundaries {etree._Element} -- parsed HDMap boundaries node

    Returns:
        _laneSection {etree._Element} -- parsed HDMap laneSection node
    """
    _laneSection = etree.Element("laneSection")
    singleSide = "false" if laneSection.get(
        "singleSide") == "false" else "true"  # NOTE: singleSide is not parsed in apollo's source code
    _laneSection.set("singleSide", singleSide)

    boundaries_empty = etree.Element("boundaries")  # for debugging
    _laneSection.append(boundaries_empty)

    return _laneSection

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
import pyproj


def geometry_parser(*args):
    """create HDMap geometry node

    Arguments:
            geometry {etree._Element} -- geometry node of OpenDRIVE
            pointSet {etree._Element} -- pointSet node parsed from OpenDRIVE geometry
            xodrproj {Projection} -- Geoprojection from the opendrive file
            apolloproj {Projection} -- Target Apollo geoprojection

            OR

            sOffset {str/float}
            x {str/float}
            y {str/float}
            z {str/float}
            length {str/float}
            pointSet {etree._Element}
            xodrproj, apolloproj -- not needed

    Raises:
        ValueError: for debugging

    Returns:
        geometry {etree._Element} -- parsed geometry node with attributes
    """
    geometry = etree.Element("geometry")

    if len(args) == 4 and isinstance(args[0], etree._Element):
        xodrproj, apolloproj = args[2], args[3]
        latlong = pyproj.transform(xodrproj, apolloproj, float(
            args[0].get("x")), float(args[0].get("y")))
        geometry.set("sOffset", args[0].get("s"))
        geometry.set("x", to_str(latlong[1]))  # implicit xy flip!
        geometry.set("y", to_str(latlong[0]))
        geometry.set("z", to_str(0))
        geometry.set("length", args[0].get("length"))
        geometry.append(args[1])

    elif len(args) == 5:
        # not needed here, TODO refactor
        xodrproj, apolloproj = args[3], args[4]
        # take coordinates from the point lists, these are already in wgs84
        coord = list(map(args[2][0].get, coordStrings))
        geometry.set("sOffset", args[0] if isinstance(
            args[0], str) else to_str(args[0]))
        geometry.set("x", coord[0])
        geometry.set("y", coord[1])
        geometry.set("z", coord[2])
        geometry.set("length", args[1] if isinstance(
            args[1], str) else to_str(args[1]))
        geometry.append(args[2])

    else:
        raise ValueError

    return geometry

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

from adaptor.common_utils import laneSectionStrings, supportedLaneTypes
from converter import opendrive2apollo_converter

import numpy as np
from lxml import etree
import unittest
unittest.TestLoader.sortTestMethodsUsing = None

testMapPath = "./modules/test_maps/Crossing8Course.xodr"
discret = 0.5


class tests(unittest.TestCase):
    ClassIsSetup = False  # converter is constructed only once

    def setUp(self):
        if not self.ClassIsSetup:
            self.setUpConverter()
            self.__class__.ClassIsSetup = True

    def setUpConverter(self):
        unittest.TestCase.setUp(self)
        self.__class__.c = opendrive2apollo_converter(discret, debug=True)
        self.__class__.c.read_opendrive_map(testMapPath)
        self.__class__.c.run()

        # some attribs changed in the orignal tree, cuased by appending
        self.__class__.c.read_opendrive_map(testMapPath)

    def test_load_map(self):
        self.assertIsNotNone(self.__class__.c.root)
        self.assertIsNotNone(self.__class__.c._root)
        self.assertEqual(self.__class__.c._root.tag, "OpenDRIVE")

    def test_map_integrity(self):
        self.assertIsNotNone(self.__class__.c._root.find("header"))
        self.assertEqual(len(self.__class__.c.root.findall("road")), len(
            self.__class__.c._root.findall("road")))
        self.assertEqual(len(self.__class__.c.root.findall("junction")), len(
            self.__class__.c._root.findall("junction")))

    def assertGeometryWithAttribsEqual(self, g, _g):
        self.assertIsNotNone(g)
        self.assertIsNotNone(_g)

        if g.get("s") is not None:
            self.assertEqual(g.get("s"), _g.get("sOffset"))
        elif g.get("sOffset") is not None:
            self.assertEqual(g.get("sOffset"), _g.get("sOffset"))
        else:
            self.fail()
        self.assertEqual(g.get("x"), _g.get("x"))
        self.assertEqual(g.get("y"), _g.get("y"))
        self.assertIsNotNone(_g.get("z"))
        self.assertEqual(g.get("length"), _g.get("length"))

        self.assertGreaterEqual(len(_g.findall("pointSet")), 1)
        self.assertPointSet(_g.find("pointSet"))

    def assertGeometryWithAttribs(self, g):
        self.assertIsNotNone(g)
        self.assertIsNotNone(g.get("sOffset"))
        self.assertIsNotNone(g.get("x"))
        self.assertIsNotNone(g.get("y"))
        self.assertIsNotNone(g.get("z"))
        self.assertIsNotNone(g.get("length"))

        self.assertGreaterEqual(len(g.findall("pointSet")), 1)
        self.assertPointSet(g.find("pointSet"))

    def assertGeometryWithoutAttribs(self, g):
        self.assertIsNotNone(g)

        self.assertGreaterEqual(len(g.findall("pointSet")), 1)
        self.assertPointSet(g.find("pointSet"))

    def assertPointSet(self, pointSet):
        self.assertIsNotNone(pointSet)
        self.assertGreaterEqual(len(pointSet), 2)

        for i in range(len(pointSet)-1):
            p1 = np.array([float(pointSet[i].get("x")),
                           float(pointSet[i].get("y")),
                           float(pointSet[i].get("z"))])
            p2 = np.array([float(pointSet[i+1].get("x")),
                           float(pointSet[i+1].get("y")),
                           float(pointSet[i+1].get("z"))])
            self.assertLessEqual(round(np.linalg.norm(p1-p2), 7), discret+0.21)

    def asserAssociates(self, associates):
        self.assertIsNotNone(associates)
        self.assertGreaterEqual(len(associates), 2)

        # too slow to run
        # for i in range(len(associates)-1):
        #     s1=float(associates[i].get("sOffset"))
        #     s2=float(associates[i+1].get("sOffset"))
        #     self.assertLessEqual(round(s2-s1,7),discret)

    def assertJunctionOutline(self, outline):
        self.assertIsNotNone(outline)
        self.assertGreaterEqual(len(outline), 3)
        # corner coordinate can also been checked

    def assertLink(self, link, _link):
        if link is not None and len(link) >= 1:
            self.assertEqual(len(link), len(_link))
            for cessor, _cessor in zip(link, _link):
                self.assertEqual(cessor.attrib, _cessor.attrib)

    def test_junctions_attribs(self):
        for j, _j in zip(self.__class__.c.root.iterfind("junction"), self.__class__.c._root.iterfind("junction")):
            self.assertEqual(j.get("id"), _j.get("id"))

            self.assertEqual(len(_j.findall("outline")), 1)
            self.assertJunctionOutline(_j.find("outline"))

            for c, _c in zip(j.iterfind("connection"), _j.iterfind("connection")):
                self.assertEqual(c.attrib, _c.attrib)
                for l, _l in zip(c.iterfind("laneLink"), _c.iterfind("laneLink")):
                    self.assertEqual(l.attrib, _l.attrib)

    def test_road_attribs(self):
        for r, _r in zip(self.__class__.c.root.iterfind("road"), self.__class__.c._root.iterfind("road")):
            self.assertEqual(r.get("name"), _r.get("name"))
            self.assertEqual(r.get("id"), _r.get("id"))
            self.assertEqual(r.get("junction"), _r.get("junction"))

            self.assertEqual(len(r.findall("link")), len(_r.findall("link")))
            link = r.find("link")
            _link = _r.find("link")
            self.assertLink(link, _link)

            self.assertEqual(len(r.findall("planView")),
                             len(_r.findall("routeView")))
            planView = r.find("planView")
            routeView = _r.find("routeView")

            self.assertEqual(len(planView), len(routeView))
            for g, _g in zip(planView, routeView):
                self.assertGeometryWithAttribsEqual(g, _g)

            self.assertEqual(len(r.findall("lanes")), len(_r.findall("lanes")))
            lanes = r.find("lanes")
            _lanes = _r.find("lanes")

            self.assertEqual(len(lanes.findall("laneSection")),
                             len(_lanes.findall("laneSection")))
            laneSection = lanes.find("laneSection")
            _laneSection = _lanes.find("laneSection")

            self.assertEqual(len(_laneSection.findall("boundaries")), 1)
            _boundaries = _laneSection.find("boundaries")

            for _boundary in _boundaries:
                self.assertTrue(_boundary.get("type") == "leftBoundary" or
                                _boundary.get("type") == "rightBoundary")
                self.assertEqual(len(_boundary.findall("geometry")), 1)
                self.assertGeometryWithoutAttribs(_boundary.find("geometry"))

            for s in laneSectionStrings:
                self.assertEqual(len(laneSection.findall(s)),
                                 len(_laneSection.findall(s)))

    def test_lane_attribs(self):
        for r, _r in zip(self.__class__.c.root.iterfind("road"), self.__class__.c._root.iterfind("road")):
            # print("road: ", r.get("id"))
            lanes = r.find("lanes")
            _lanes = _r.find("lanes")
            laneSection = lanes.find("laneSection")
            _laneSection = _lanes.find("laneSection")

            routeView = _r.find("routeView")

            for section, _section, laneSectionString in zip(map(laneSection.find, laneSectionStrings), map(_laneSection.find, laneSectionStrings), laneSectionStrings):
                if section is None:
                    self.assertIsNone(_section)
                    continue

                for lane, _lane in zip(section, _section):
                    # print("lane: ", lane.get("id"))
                    self.assertEqual(lane.get("id"), _lane.get("id"))
                    self.assertIsNotNone(_lane.get("uid"))
                    self.assertIn(_lane.get("type"), supportedLaneTypes)
                    self.assertIsNotNone(_lane.get("turnType"))

                    link = lane.find("link")
                    _link = _lane.find("link")
                    self.assertLink(link, _link)

                    """
                    Tests below accord to 
                    1. the master thesis (Gran-ChristofferWilhelm_2019_Master_NAP_HDMaps.pdf)
                    2. apollo map generator's source code
                    """

                    if laneSectionString == "center":
                        border = _lane.find("border")
                        for g, bg in zip(routeView, border):
                            self.assertGeometryWithAttribsEqual(g, bg)
                    else:
                        self.assertEqual(len(_lane.findall("centerLine")), 1)
                        centerLine = _lane.find("centerLine")
                        self.assertEqual(len(routeView), len(centerLine))

                        border = _lane.find("border")
                        for g in border:
                            self.assertGeometryWithAttribs(g)

                        self.assertEqual(
                            len(_lane.findall("sampleAssociates")), 1)
                        self.asserAssociates(_lane.find("sampleAssociates"))

                        self.assertEqual(
                            len(_lane.findall("roadSampleAssociations")), 1)
                        self.asserAssociates(
                            _lane.find("roadSampleAssociations"))


if __name__ == "__main__":
    unittest.main(verbosity=2)

// Copyright (c) 2020 fortiss GmbH
//
// Authors: Julian Bernhard, Klemens Esterle, Patrick Hart and
// Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef BARK_COMMONS_PARAMS_PARAMS_HPP_
#define BARK_COMMONS_PARAMS_PARAMS_HPP_

#include <boost/geometry.hpp>


namespace bark {
namespace commons {



class Params {
 public:
 using Point2d_test = boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;
  Params();
  double Distance(const Point2d_test& p1, const Point2d_test& p2);
};

}
}

#endif //BARK_COMMONS_PARAMS_PARAMS_HPP_
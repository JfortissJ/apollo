/******************************************************************************
 * Copyright 2021 fortiss GmbH
 * Authors: Tobias Kessler, Klemens Esterle
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/


#include "modules/planning/planner/miqp/trajectory_smoother_nlopt.h"

#include <cmath>

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(TrajectorySmootherNLOpt, Constructor) {
  TrajectorySmootherNLOpt tsm = TrajectorySmootherNLOpt();
  EXPECT_NEAR(-1.0, -1.0, 1e-5);
}

// TEST 1: Integration Model

// Test 2: Constraint Checking

// Test 3: Optimization

}  // namespace planning
}  // namespace apollo

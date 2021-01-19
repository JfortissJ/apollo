// Copyright (c) 2020 Tobias Kessler, Klemens Esterle
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// we need to wrap: in this order
// MiqpPlanner planner = MiqpPlanner(settings, envPoly);
// planner_.UpdateConvexifiedMap(envPoly_);
// planner.AddObstacle(predictedTraj, obstacleShape);

typedef void* CMiqpPlanner;

extern "C" CMiqpPlanner NewCMiqpPlanner();

extern "C" void DelCMiqpPlanner(CMiqpPlanner c_miqp_planner);

extern "C" int AddCarCMiqpPlanner(CMiqpPlanner c_miqp_planner,
                                  double initial_state_in[], double ref_in[],
                                  const int ref_size, double vDes,
                                  const double timestep);

extern "C" bool PlanCMiqpPlanner(CMiqpPlanner c_miqp_planner,
                                 const double timestep);

extern "C" void UpdateCarCMiqpPlanner(CMiqpPlanner c_miqp_planner, int idx,
                                      double initial_state_in[],
                                      double ref_in[], const int ref_size,
                                      const double timestep);

extern "C" void GetCTrajectoryCMiqpPlanner(CMiqpPlanner c_miqp_planner, int idx,
                                           double start_time,
                                           double* trajectory, int& size);

extern "C" void ActivateDebugFileWriteCMiqpPlanner(CMiqpPlanner c_miqp_planner,
                                                   char path[], char name[]);
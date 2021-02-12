// Copyright (c) 2020 Tobias Kessler, Klemens Esterle
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

// we need to wrap: in this order
// MiqpPlanner planner = MiqpPlanner(settings, envPoly);
// planner_.UpdateConvexifiedMap(envPoly_);
// planner.AddObstacle(predictedTraj, obstacleShape);

#include "miqp_planner_settings.h"

#define TRAJECTORY_TIME_IDX 0
#define TRAJECTORY_X_IDX 1
#define TRAJECTORY_Y_IDX 2
#define TRAJECTORY_VX_IDX 3
#define TRAJECTORY_VY_IDX 4
#define TRAJECTORY_AX_IDX 5
#define TRAJECTORY_AY_IDX 6
#define TRAJECTORY_UX_IDX 7
#define TRAJECTORY_UY_IDX 8
#define TRAJECTORY_SIZE 9

typedef void* CMiqpPlanner;

extern "C" {

CMiqpPlanner NewCMiqpPlanner();

CMiqpPlanner NewCMiqpPlannerSettings(MiqpPlannerSettings settings);

void DelCMiqpPlanner(CMiqpPlanner c_miqp_planner);

int AddCarCMiqpPlanner(CMiqpPlanner c_miqp_planner, double initial_state_in[],
                       double ref_in[], const int ref_size, double vDes,
                       double deltaSDes, const double timestep,
                       const bool track_reference_positions);

bool PlanCMiqpPlanner(CMiqpPlanner c_miqp_planner, const double timestep);

void UpdateCarCMiqpPlanner(CMiqpPlanner c_miqp_planner, int idx,
                           double initial_state_in[], double ref_in[],
                           const int ref_size, const double timestep,
                           bool track_reference_positions);

void GetCTrajectoryCMiqpPlanner(CMiqpPlanner c_miqp_planner, int idx,
                                double start_time, double* trajectory,
                                int& size);

void ActivateDebugFileWriteCMiqpPlanner(CMiqpPlanner c_miqp_planner,
                                        char path[], char name[]);

int GetNCMiqpPlanner(CMiqpPlanner c_miqp_planner);

float GetTsCMiqpPlanner(CMiqpPlanner c_miqp_planner);

void GetRawCMiqpTrajectoryCMiqpPlanner(CMiqpPlanner c_miqp_planner, int carIdx,
                                       double start_time, double* trajectory,
                                       int& size);

bool UpdateConvexifiedMapCMiqpPlaner(CMiqpPlanner c_miqp_planner,
                                     double poly_pts[], const int poly_size);

void UpdateDesiredVelocityCMiqpPlanner(CMiqpPlanner c_miqp_planner,
                                       const int carIdx, const double vDes,
                                       const double deltaSDes);

int AddObstacleCMiqpPlanner(CMiqpPlanner c_miqp_planner, double min_x[],
                            double max_x[], double min_y[], double max_y[],
                            int& size);

void UpdateObstacleCMiqpPlanner(CMiqpPlanner c_miqp_planner, int id,
                                double min_x[], double max_x[], double min_y[],
                                double max_y[], int& size);

void RemoveAllObstaclesCMiqpPlanner(CMiqpPlanner c_miqp_planner);

}

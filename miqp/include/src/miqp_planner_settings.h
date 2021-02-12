// Copyright (c) 2020 Tobias Kessler, Klemens Esterle
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MIQP_PLANNER_SETTINGS_HEADER
#define MIQP_PLANNER_SETTINGS_HEADER

enum MiqpPlannerWarmstartType {
    NO_WARMSTART = 0,
    RECEDING_HORIZON_WARMSTART = 1,
    LAST_SOLUTION_WARMSTART = 2,
    BOTH_WARMSTART_STRATEGIES = 3
  };

struct MiqpPlannerSettings {
  int nr_regions;
  int nr_steps;
  int nr_neighbouring_possible_regions;
  float ts;
  int precision;
  float constant_agent_safety_distance_slack;
  float minimum_region_change_speed;
  float lambda;
  float wheelBase;
  float collisionRadius;
  float slackWeight;
  float jerkWeight;
  float positionWeight;
  float velocityWeight;
  float acclerationWeight;
  float simplificationDistanceMap;
  float bufferReference;
  float refLineInterpInc;
  float scaleVelocityForReferenceLongerHorizon;
  float max_solution_time;
  float relative_mip_gap_tolerance;
  int mipdisplay;
  int mipemphasis;
  float relobjdif;
  int cutpass;
  int probe;
  int repairtries;
  int rinsheur;
  int varsel;
  int mircuts;
  const char* cplexModelpath;
  bool useSos;
  bool useBranchingPriorities;
  MiqpPlannerWarmstartType warmstartType;
};

#endif  // MIQP_PLANNER_SETTINGS_HEADER

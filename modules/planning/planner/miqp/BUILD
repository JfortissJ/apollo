load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "miqp_planner",
    srcs = ["miqp_planner.cc"],
    hdrs = ["miqp_planner.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/math:path_matcher",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/constraint_checker",
        "//modules/planning/constraint_checker:collision_checker",
        "//modules/planning/planner",
        "//modules/planning/planner/lattice:lattice_planner",
        "//modules/planning/proto:planning_proto",
        ":trajectory_smoother_nlopt",
        "@miqp_planner//:miqp_c_api",
        "@cplex",
    ],
    data = ["@miqp_planner//:cplex_models_filegroup"],
)

cc_library(
    name = "trajectory_smoother_nlopt",
    srcs = ["trajectory_smoother_nlopt.cc"],
    hdrs = ["trajectory_smoother_nlopt.h"],
    deps = [
        "//cyber/common:log",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/common/time:time",
        "//cyber/logger:logger_util",
        "@nlopt",
        "@eigen",
    ]
)

filegroup(
    name = "miqp_testdata",
    srcs = glob([
        "miqp_testdata/**",
    ]),
)

cc_test(
    name = "trajectory_smoother_nlopt_test",
    srcs = ["trajectory_smoother_nlopt_test.cc"],
    deps = ["@gtest//:main",
            ":trajectory_smoother_nlopt",
            "//modules/common/math:geometry"],
    data = [":miqp_testdata"]
)

cpplint()

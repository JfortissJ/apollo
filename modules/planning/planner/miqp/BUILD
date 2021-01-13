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
        "@bark//:core",
    ],
)

cpplint()

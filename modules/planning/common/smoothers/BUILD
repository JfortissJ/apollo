load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "smoother",
    srcs = ["smoother.cc"],
    hdrs = ["smoother.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planning_proto",
    ],
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

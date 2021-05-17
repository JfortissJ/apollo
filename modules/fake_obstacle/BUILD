load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fake_obstacle_component_lib",
    srcs = ["fake_obstacle_component.cc"],
    hdrs = ["fake_obstacle_component.h"],
    copts = [
        "-DMODULE_NAME=\\\"fake_obstacle\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common/monitor_log",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/localization/proto:localization_proto",
        "//modules/fake_obstacle/proto:fake_obstacle_proto",
        "//modules/routing/proto:routing_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/planning/reference_line:reference_line_provider",
    ],
)

cc_binary(
    name = "libfake_obstacle_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":fake_obstacle_component_lib"],
)

filegroup(
    name = "fake_obstacle_component_conf",
    srcs = glob([
        "conf/**",
    ]),
)

cpplint()

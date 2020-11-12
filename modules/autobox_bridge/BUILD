load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libautobox_bridge_component_chassis.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":autobox_bridge_component_lib_chassis"],
)

cc_library(
    name = "autobox_bridge_component_lib_chassis",
    srcs = ["autobox_bridge_component_chassis.cc"],
    hdrs = ["autobox_bridge_component_chassis.h"],
    copts = ['-DMODULE_NAME=\\"autobox_bridge\\"'],
    deps = [
        "//cyber",
        "//modules/common/monitor_log",
        "//modules/control/proto:control_command_proto",
        "//modules/autobox_bridge/proto:autobox_chassis_proto",
        "//modules/autobox_bridge/proto:autobox_bridge_conf_proto",
    ],
)

cc_binary(
    name = "libautobox_bridge_component_control.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":autobox_bridge_component_lib_control"],
)

cc_library(
    name = "autobox_bridge_component_lib_control",
    srcs = ["autobox_bridge_component_control.cc"],
    hdrs = ["autobox_bridge_component_control.h"],
    copts = ['-DMODULE_NAME=\\"autobox_bridge\\"'],
    deps = [
        "//cyber",
        "//modules/common/monitor_log",
        "//modules/control/proto:control_command_proto",
	    "//modules/autobox_bridge/proto:autobox_control_proto",
        "//modules/autobox_bridge/proto:autobox_bridge_conf_proto",
    ],
)

cc_binary(
    name = "libautobox_bridge_component_localization.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":autobox_bridge_component_lib_localization"],
)

cc_library(
    name = "autobox_bridge_component_lib_localization",
    srcs = ["autobox_bridge_component_localization.cc"],
    hdrs = ["autobox_bridge_component_localization.h"],
    copts = ['-DMODULE_NAME=\\"autobox_bridge\\"'],
    deps = [
        "//cyber",
        "//modules/common/monitor_log",
        "//modules/localization/proto:localization_proto",
	    "//modules/autobox_bridge/proto:autobox_localization_proto",
        "//modules/autobox_bridge/proto:autobox_bridge_conf_proto",
    ],
)

cc_binary(
    name = "libautobox_bridge_component_trajectory.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":autobox_bridge_component_lib_trajectory"],
)

cc_library(
    name = "autobox_bridge_component_lib_trajectory",
    srcs = ["autobox_bridge_component_trajectory.cc"],
    hdrs = ["autobox_bridge_component_trajectory.h"],
    copts = ['-DMODULE_NAME=\\"autobox_bridge\\"'],
    deps = [
        "//cyber",
        "//modules/common/monitor_log",
        "//modules/control/proto:control_command_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/autobox_bridge/proto:autobox_trajectory_proto",
        "//modules/autobox_bridge/proto:autobox_bridge_conf_proto",
    ],
)

filegroup(
    name = "ros_to_ma_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

cpplint()

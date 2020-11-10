package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "autobox_bridge_conf_proto",
    deps = [
        ":autobox_bridge_conf_proto_lib",
    ],
)

proto_library(
    name = "autobox_bridge_conf_proto_lib",
    srcs = [
        "autobox_bridge_conf.proto"
    ],
)

# Delete the suff from here on if the repacking of protobuf messages is obsolete!

cc_proto_library(
    name = "autobox_trajectory_proto",
    deps = [
        ":autobox_trajectory_proto_lib",
    ],
)

proto_library(
    name = "autobox_trajectory_proto_lib",
    srcs = [
        "autobox_trajectory.proto",
    ],
    deps = [
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "autobox_localization_proto",
    deps = [
        ":autobox_localization_proto_lib",
    ],
)

proto_library(
    name = "autobox_localization_proto_lib",
    srcs = [
        "autobox_localization.proto",
    ],
    deps = [
        "//modules/localization/proto:pose_proto_lib",
        "//modules/localization/proto:localization_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)


cc_proto_library(
    name = "autobox_control_proto",
    deps = [
        ":autobox_control_proto_lib",
    ],
)

proto_library(
    name = "autobox_control_proto_lib",
    srcs = [
        "autobox_control.proto",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "autobox_chassis_proto",
    deps = [
        ":autobox_chassis_proto_lib",
    ],
)

proto_library(
    name = "autobox_chassis_proto_lib",
    srcs = [
        "autobox_chassis.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)


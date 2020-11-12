load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libimar_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":imar_component_lib"],
)

cc_library(
    name = "imar_component_lib",
    srcs = ["imar_component.cc"],
    hdrs = ["imar_component.h"],
    copts = ['-DMODULE_NAME=\\"imar\\"'],
    deps = [
        "//cyber",
        "//modules/common/monitor_log",
        "//modules/drivers/imar:imar_ethernet_lib",
    ],
)

cc_library(
    name = "imar_ethernet_lib",
    srcs = [
        "imar_ethernet.cc",
    ],
    hdrs = [
        "imar_ethernet.h",
    ],
    deps = [
        "//cyber",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:imu_proto",
        "//modules/localization/proto:pose_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/drivers/imar/imar_sdk:imar_sdk",
        "//modules/drivers/imar/proto:imar_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers/gnss/proto:ins_proto",
        "//modules/localization/msf/common/util:localization_msf_common_util_frame_transform"
    ],
)

cpplint()

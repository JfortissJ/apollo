load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_fortuna_protocol",
    srcs = [
        "assistance_2_rpt_0xe.cc",
        "motion_rpt_0x01.cc",
        "powertrain_1_rpt_0x04.cc",
        "front_object_1_rpt_0x31.cc",
        "front_object_2_rpt_0x32.cc",
        "front_object_3_rpt_0x33.cc",
        "front_object_4_rpt_0x34.cc",
        "rear_object_1_rpt_0x41.cc",
        "rear_object_2_rpt_0x42.cc",
        "rear_object_3_rpt_0x43.cc",
        "rear_object_4_rpt_0x44.cc",
        "rear_object_5_rpt_0x45.cc",
        "rear_object_6_rpt_0x46.cc",
        "steering_rpt_0x02.cc",
    ],
    hdrs = [
        "assistance_2_rpt_0xe.h",
        "motion_rpt_0x01.h",
        "powertrain_1_rpt_0x04.h",
        "front_object_1_rpt_0x31.h",
        "front_object_2_rpt_0x32.h",        
        "front_object_3_rpt_0x33.h",        
        "front_object_4_rpt_0x34.h",        
        "rear_object_1_rpt_0x41.h",
        "rear_object_2_rpt_0x42.h",        
        "rear_object_3_rpt_0x43.h",        
        "rear_object_4_rpt_0x44.h",        
        "rear_object_5_rpt_0x45.h",        
        "rear_object_6_rpt_0x46.h",
        "steering_rpt_0x02.h",        
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

## Yet todo: unit tests!
# cc_test(
#     name = "fortuna_ca_protocol_test",
#     size = "small",
#     srcs = [
#         "assistance_2_rpt_0xe.cc",
#         "motion_rpt_0x01.cc",
#         "powertrain_1_rpt_0x04.cc",
#     ],
#     deps = [
#         "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
#         "@gtest//:main",
#     ],
# )


cpplint()

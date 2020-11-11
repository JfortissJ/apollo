load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fortuna_vehicle_factory",
    srcs = [
        "fortuna_vehicle_factory.cc",
    ],
    hdrs = [
        "fortuna_vehicle_factory.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        ":fortuna_controller",
        ":fortuna_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "fortuna_message_manager",
    srcs = [
        "fortuna_message_manager.cc",
    ],
    hdrs = [
        "fortuna_message_manager.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle/fortuna/protocol:canbus_fortuna_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "fortuna_controller",
    srcs = [
        "fortuna_controller.cc",
    ],
    hdrs = [
        "fortuna_controller.h",
    ],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        ":fortuna_message_manager",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/fortuna/protocol:canbus_fortuna_protocol",
        "//modules/common/kv_db",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@gtest",
    ],
)

#cc_test(
#    name = "fortuna_controller_test",
#    size = "small",
#    srcs = [
#        "fortuna_controller_test.cc",
#    ],
#    data = ["//modules/canbus:canbus_testdata"],
#    deps = [
#        "//modules/canbus/vehicle/fortuna:fortuna_controller",
#        "@gtest//:main",
#    ],
#)

# cc_test(
#     name = "fortuna_message_manager_test",
#     size = "small",
#     srcs = [
#         "fortuna_message_manager_test.cc",
#     ],
#     deps = [
#         "//modules/canbus/vehicle/fortuna:fortuna_message_manager",
#         "@gtest//:main",
#     ],
# )

#cc_test(
#    name = "fortuna_vehicle_factory_test",
#    size = "small",
#    srcs = ["fortuna_vehicle_factory_test.cc"],
#    deps = [
#        ":fortuna_vehicle_factory",
#        "@gtest//:main",
#    ],
#)

cpplint()

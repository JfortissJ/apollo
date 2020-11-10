load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_library
cc_library(
    name = "marco",
    hdrs = ["macro.h"],
)

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_library
cc_library(
    name = "bridge_header",
    hdrs = [
        "bridge_header.h",
        "bridge_header_item.h",
    ],
)

cc_library(
    name = "bridge_proto_diserialized_buf",
    hdrs = ["bridge_proto_diserialized_buf.h"],
    deps = [
        ":bridge_header",
        ":marco",
        "//modules/autobox_gateway/proto:autobox_adc_trajectrory_proto_lib",
        "//modules/autobox_gateway/proto:autobox_control_proto_lib",
        "//modules/autobox_gateway/proto:autobox_localization_proto_lib",
        "//modules/control/proto:control_proto_lib",
        "//modules/planning/proto:planning_proto_lib",
        "//third_party/protobuf-c:protobufc",
    ],
)

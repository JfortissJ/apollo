package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "fake_obstacle_proto",
    deps = [
        ":fake_obstacle_proto_lib",
    ],
)

proto_library(
    name = "fake_obstacle_proto_lib",
    srcs = [
        "fake_obstacle_conf.proto",
    ],
    deps = [
        "//modules/perception/proto:perception_proto_lib",
    ],
)

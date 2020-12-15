py_binary(
    name = "converter",
    srcs = ["converter.py"],
    visibility = ["//visibility:public"],
    data = [
        "//modules/test_maps:test_maps",
    ],
    deps = [
            "//modules/adaptor:common_utils",
            "//modules/adaptor:geometry_parser",
            "//modules/adaptor:geometry_utils",
            "//modules/adaptor:lane_parser",
            "//modules/adaptor:road_parser",
            "//modules/adaptor:laneSection_parser",
            "//modules/visualization_tools:lane_connection_visualization",
            "//modules/visualization_tools:map_visualization",
          ],
)

py_test(
    name = "unit_test",
    srcs = ["unit_test.py"],
    visibility = ["//visibility:public"],
    data = [
        "//modules/test_maps:test_maps",
    ],
    deps = [
            "//modules/adaptor:common_utils",
            "//modules:converter",
          ],
)
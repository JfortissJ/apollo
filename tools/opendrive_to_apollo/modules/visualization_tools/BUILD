py_library(
    name = "map_visualization",
    srcs = ["map_visualization.py"],
    visibility = ["//visibility:public"],
)


py_binary(
    name = "lane_connection_visualization",
    srcs = ["lane_connection_visualization.py"],
    deps = [
            "//modules/adaptor:common_utils",
    ],
    data = [
        "//:root_dir_files",
    ],
    visibility = ["//visibility:public"],
)

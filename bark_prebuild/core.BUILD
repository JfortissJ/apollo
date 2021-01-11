package(default_visibility = ["//visibility:public"])

cc_library(
    name = "core",
    copts = [],
    hdrs = glob([
        "*.h",
    ]),
    includes = [
        ".",
        "commons/params/*.h",
    ],
    linkopts = [
        "-L/apollo/bark_prebuild",
    ],
    visibility = ["//visibility:public"],
)
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "core",
    copts = [],
    # hdrs = glob([
    # #    "*.h",
    #     "commons/params/params.h"
    # ]),
    includes = [
        ".",
    #     "commons/params/*.h",
    ],
    linkopts = [
        "-L/apollo/bark/lib",
        "-lcorec"
    ],
    visibility = ["//visibility:public"],
)
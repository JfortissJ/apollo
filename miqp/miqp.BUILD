package(default_visibility = ["//visibility:public"])

cc_library(
    name = "miqp_c_api",
    copts = [],
    includes = ["include/."],
    srcs = ["lib/libmiqp_planner_c_api.so"],
    visibility = ["//visibility:public"],
)

filegroup(
   name="cplex_models_filegroup",
   srcs=glob(["cplex_modfiles/*.mod"]),
   visibility = ["//visibility:public"],
)
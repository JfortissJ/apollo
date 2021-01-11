licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "boost_bark",
    copts = ["-fPIC"],
    includes = ["."],
    linkopts = [
        "-L/usr/local/apollo/boost_bark/lib",
        "-lboost_system",
        "-lboost_filesystem",
        "-lboost_program_options",
        "-lboost_thread",
        "-lboost_signals",
        "-lpthread",
        "-lstdc++",
        "-lbacktrace",
        "-ldl",
    ],
)

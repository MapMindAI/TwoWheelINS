load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

exports_files(["LICENSE"])

# UDU (Bierman/Thornton) Kalman filter with the built-in miniblas; C only, static memory.
cc_library(
    name = "kfcore",
    srcs = [
        "kalman_udu.c",
        "linalg.c",
        "miniblas.c",
    ],
    hdrs = [
        "kalman_udu.h",
        "linalg.h",
        "miniblas.h",
    ],
    includes = ["."],
    linkopts = ["-lm"],
)

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ikfom_toolkit",
    hdrs = glob(["**/*.h*"]),
    deps = [
        "@glog",
        "@eigen",
        "@sophus",
    ],
)

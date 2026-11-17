load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kfcore_ins",
    srcs = ["kfcore_ins.c"],
    hdrs = ["kfcore_ins.h"],
    deps = [
        "//common/math:quaternion",
        "//third_party/kfcore",
    ],
)

cc_test(
    name = "kfcore_ins_test",
    srcs = ["kfcore_ins_test.cc"],
    deps = [
        ":kfcore_ins",
        "@eigen",
        "@googletest//:gtest_main",
        "@sophus",
    ],
)

cc_library(
    name = "ins_fusion",
    srcs = ["ins_fusion.c"],
    hdrs = ["ins_fusion.h"],
    deps = [
        ":kfcore_ins",
        "//common/math:geodesy",
        "//common/math:quaternion",
        "//common/sensor:gps_accuracy",
    ],
)

cc_binary(
    name = "ins_fusion_main",
    srcs = ["ins_fusion_main.cc"],
    deps = [
        ":ins_fusion",
        "//common/math:geodesy",
        "//common/reader:binary_record_reader",
        "//common/reader:txt_record_reader",
        "@eigen",
        "@gflags",
    ],
)

cc_test(
    name = "ins_fusion_test",
    srcs = ["ins_fusion_test.cc"],
    deps = [
        ":ins_fusion",
        "//common/math:geodesy",
        "@eigen",
        "@googletest//:gtest_main",
        "@sophus",
    ],
)

cc_binary(
    name = "run_dataset_main",
    srcs = ["run_dataset_main.cc"],
    deps = [
        ":ins_fusion",
        "//common/math:geodesy",
        "//common/reader:benchmark_dataset_reader",
        "//common/simulation:gps_noise",
        "@eigen",
        "@gflags",
    ],
)

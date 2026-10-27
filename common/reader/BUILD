load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "txt_record_reader",
    srcs = ["txt_record_reader.cc"],
    hdrs = ["txt_record_reader.h"],
    deps = [
        "//common/sensor:gps_data",
        "//common/sensor:imu_data",
        "//common/sensor:wheel_speed_data",
        "@eigen",
    ],
)

cc_test(
    name = "txt_record_reader_test",
    srcs = ["txt_record_reader_test.cc"],
    deps = [
        ":txt_record_reader",
        "@googletest//:gtest_main",
    ],
)

cc_library(
    name = "benchmark_dataset_reader",
    srcs = ["benchmark_dataset_reader.cc"],
    hdrs = ["benchmark_dataset_reader.h"],
    deps = [
        "//common/sensor:imu_data",
        "@eigen",
    ],
)

cc_test(
    name = "benchmark_dataset_reader_test",
    srcs = ["benchmark_dataset_reader_test.cc"],
    deps = [
        ":benchmark_dataset_reader",
        "@googletest//:gtest_main",
    ],
)

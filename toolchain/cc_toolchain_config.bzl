load("@rules_cc//cc:action_names.bzl", "ACTION_NAMES")
load("@rules_cc//cc/common:cc_common.bzl", "cc_common")
load("@rules_cc//cc/toolchains:cc_toolchain_config_info.bzl", "CcToolchainConfigInfo")
load(
    "@rules_cc//cc:cc_toolchain_config_lib.bzl",
    "feature",
    "flag_group",
    "flag_set",
    "tool_path",
)

all_compile_actions = [
    ACTION_NAMES.c_compile,
    ACTION_NAMES.cpp_compile,
    ACTION_NAMES.cpp_header_parsing,
    ACTION_NAMES.cpp_module_compile,
    ACTION_NAMES.preprocess_assemble,
]

all_link_actions = [
    ACTION_NAMES.cpp_link_executable,
    ACTION_NAMES.cpp_link_dynamic_library,
    ACTION_NAMES.cpp_link_nodeps_dynamic_library,
]

def _impl(ctx):
    tool_paths = [
        tool_path(
            name = "gcc",
            path = "/usr/bin/aarch64-linux-gnu-gcc",
        ),
        tool_path(
            name = "g++",
            path = "/usr/bin/aarch64-linux-gnu-g++",
        ),
        tool_path(
            name = "ld",
            path = "/usr/bin/aarch64-linux-gnu-ld",
        ),
        tool_path(
            name = "ld.gold",
            path = "/usr/bin/aarch64-linux-gnu-ld.gold",
        ),
        tool_path(
            name = "ar",
            path = "/usr/bin/aarch64-linux-gnu-ar",
        ),
        tool_path(
            name = "cpp",
            path = "/usr/bin/aarch64-linux-gnu-cpp",
        ),
        tool_path(
            name = "gcov",
            path = "/usr/bin/aarch64-linux-gnu-gcov",
        ),
        tool_path(
            name = "nm",
            path = "/usr/bin/aarch64-linux-gnu-nm",
        ),
        tool_path(
            name = "objdump",
            path = "/usr/bin/aarch64-linux-gnu-objdump",
        ),
        tool_path(
            name = "strip",
            path = "/usr/bin/aarch64-linux-gnu-strip",
        ),
    ]

    features = [
        # Bazel enables the feature matching -c (opt/dbg); without these, -c opt adds no flags.
        feature(
            name = "opt",
            flag_sets = [
                flag_set(
                    actions = all_compile_actions,
                    flag_groups = [flag_group(flags = [
                        "-O2",
                        "-DNDEBUG",
                        "-ffunction-sections",
                        "-fdata-sections",
                    ])],
                ),
                flag_set(
                    actions = all_link_actions,
                    flag_groups = [flag_group(flags = ["-Wl,--gc-sections"])],
                ),
            ],
        ),
        feature(
            name = "dbg",
            flag_sets = [
                flag_set(
                    actions = all_compile_actions,
                    flag_groups = [flag_group(flags = ["-g", "-O0"])],
                ),
            ],
        ),
        # Lets --features=external_include_paths include external repos with -isystem.
        feature(
            name = "external_include_paths",
            flag_sets = [
                flag_set(
                    actions = all_compile_actions,
                    flag_groups = [
                        flag_group(
                            flags = ["-isystem", "%{external_include_paths}"],
                            iterate_over = "external_include_paths",
                            expand_if_available = "external_include_paths",
                        ),
                    ],
                ),
            ],
        ),
        feature(
            name = "default_linker_flags",
            enabled = True,
            flag_sets = [
                flag_set(
                    actions = all_link_actions,
                    flag_groups = [
                        flag_group(
                            flags = [
                                "-lstdc++",
                                "-lm",
                                "-L/usr/aarch64-linux-gnu",
                                "-L/usr/aarch64-linux-gnu/lib",
                                "-L/usr/lib/aarch64-linux-gnu",
                                "-L/usr/lib/aarch64-linux-gnu/blas",
                                "-L/usr/lib/aarch64-linux-gnu/lapack",
                                "-L/usr/local/lib/aarch64-linux-gnu",
                                "-Wl,-rpath-link,/usr/lib/aarch64-linux-gnu",
                                "-Wl,-rpath-link,/usr/lib/aarch64-linux-gnu/blas",
                                "-Wl,-rpath-link,/usr/lib/aarch64-linux-gnu/lapack",
                                "-Wl,-rpath-link,/usr/local/lib/aarch64-linux-gnu",
                            ],
                        ),
                    ],
                ),
            ],
        ),
    ]

    return cc_common.create_cc_toolchain_config_info(
        abi_libc_version = "unknown",
        abi_version = "unknown",
        compiler = "aarch64-linux-gnu",
        ctx = ctx,
        cxx_builtin_include_directories = [
            "/usr/local/include",
            "/usr/aarch64-linux-gnu/include/",
            "/usr/lib/aarch64-linux-gnu/",
            "/usr/lib/gcc/aarch64-linux-gnu/",
            "/usr/lib/gcc-cross/aarch64-linux-gnu/",
            "/usr/include",
        ],
        features = features,
        host_system_name = "local",
        target_cpu = "aarch64",
        target_libc = "unknown",
        target_system_name = "local",
        tool_paths = tool_paths,
        toolchain_identifier = "local",
    )

cc_toolchain_config = rule(
    attrs = {},
    provides = [CcToolchainConfigInfo],
    implementation = _impl,
)

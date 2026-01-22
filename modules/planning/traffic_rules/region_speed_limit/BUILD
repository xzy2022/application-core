load("//tools:apollo.bzl", "cyber_plugin_description")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "region_speed_limit_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libregion_speed_limit.so",
    srcs = [
        "region_speed_limit.cc",
    ],
    hdrs = [
        "region_speed_limit.h",
    ],
    # 保持 buildtool 生成的文件名
    description = ":plugin_region_speed_limit_description.xml",
    deps = [
        "//cyber",
        # 必须补上下面这个依赖，否则编译不过
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/traffic_rules/region_speed_limit/proto:region_speed_limit_proto",
    ],
)

apollo_package()

cpplint()
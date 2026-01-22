/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/******************************************************************************
 * @file region_speed_limit.h
 *****************************************************************************/

 #pragma once

 #include <memory>
 #include <string>
 
 #include "cyber/plugin_manager/plugin_manager.h"
 #include "modules/common/status/status.h"
 #include "modules/planning/planning_interface_base/traffic_rules_base/traffic_rule.h"
 #include "modules/planning/traffic_rules/region_speed_limit/proto/region_speed_limit.pb.h"
 
 namespace apollo {
 namespace planning {
 
 class RegionSpeedLimit : public TrafficRule {
  public:
   /**
    * @brief 初始化插件，加载配置和注入器
    */
   bool Init(const std::string& name,
             const std::shared_ptr<DependencyInjector>& injector) override;
 
   virtual ~RegionSpeedLimit() = default;
 
   /**
    * @brief 核心业务逻辑：应用限速规则
    */
   common::Status ApplyRule(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) override;
 
   /**
    * @brief 重置状态（本插件为无状态设计，故为空）
    */
   void Reset() override {}
 
  private:
   // 对应 Proto 定义的配置对象
   RegionSpeedLimitConfig config_;
 };
 
 /**
  * @brief 插件注册宏，将此类注册到 Cyber 框架中
  * 参数1：插件类名（带完整命名空间）
  * 参数2：基类名（带完整命名空间）
  */
 CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::RegionSpeedLimit,
                                      apollo::planning::TrafficRule)
 
 }  // namespace planning
 }  // namespace apollo
 Build文件
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
 region_speed_limit.proto
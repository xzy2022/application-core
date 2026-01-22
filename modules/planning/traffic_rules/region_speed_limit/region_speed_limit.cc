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
 * @file region_speed_limit.cc
 *****************************************************************************/

 #include "modules/planning/traffic_rules/region_speed_limit/region_speed_limit.h"

 #include <memory>
 #include <string>
 #include <vector>
 
 namespace apollo {
 namespace planning {
 
 using apollo::common::Status;
 using apollo::hdmap::PathOverlap;
 
 /**
  * @brief 初始化插件
  */
 bool RegionSpeedLimit::Init(
     const std::string& name,
     const std::shared_ptr<DependencyInjector>& injector) {
   // 1. 调用基类的初始化逻辑
   if (!TrafficRule::Init(name, injector)) {
     return false;
   }
   // 2. 加载该任务特有的配置 (RegionSpeedLimitConfig)
   return TrafficRule::LoadConfig<RegionSpeedLimitConfig>(&config_);
 }
 
 /**
  * @brief 执行规则逻辑
  */
 Status RegionSpeedLimit::ApplyRule(
     Frame* const frame, 
     ReferenceLineInfo* const reference_line_info) {
   
   // 获取可修改的参考线对象
   ReferenceLine* reference_line = reference_line_info->mutable_reference_line();
   
   // 从地图路径中获取所有 PNC 路口（PNC Junction）的重叠区域
   const std::vector<PathOverlap>& pnc_junction_overlaps = 
       reference_line_info->reference_line().map_path().pnc_junction_overlaps();
 
   // 遍历每一个路口区域
   for (const auto& pnc_junction_overlap : pnc_junction_overlaps) {
     // 为参考线添加限速
     // 参数：起始位置(s) - 前向缓冲，结束位置(s) + 后向缓冲，限制速度
     reference_line->AddSpeedLimit(
         pnc_junction_overlap.start_s - config_.forward_buffer(), 
         pnc_junction_overlap.end_s + config_.backward_buffer(), 
         config_.limit_speed());
   }
 
   return Status::OK();
 }
 
 }  // namespace planning
 }  // namespace apollo
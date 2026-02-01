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

#include "modules/planning/tasks/lane_follow_path/lane_follow_path.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <fstream>  // Added for file logging

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/util/print_debug_info.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

bool LaneFollowPath::Init(const std::string& config_dir,
                          const std::string& name,
                          const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  return Task::LoadConfig<LaneFollowPathConfig>(&config_);
}

apollo::common::Status LaneFollowPath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->path_data().Empty() ||
      reference_line_info->path_reusable()) {
    ADEBUG << "Skip this time path empty:"
           << reference_line_info->path_data().Empty()
           << "path reusable: " << reference_line_info->path_reusable();
    return Status::OK();
  }
  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<PathData> candidate_path_data;

  GetStartPointSLState();
  if (!DecidePathBounds(&candidate_path_boundaries)) {
    AERROR << "Decide path bound failed";
    return Status::OK();
  }
  if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
    AERROR << "Optmize path failed";
    return Status::OK();
  }
  if (!AssessPath(&candidate_path_data,
                  reference_line_info->mutable_path_data())) {
    AERROR << "Path assessment failed";
  }

  return Status::OK();
}

bool LaneFollowPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  boundary->emplace_back();
  auto& path_bound = boundary->back();
  std::string blocking_obstacle_id = "";
  std::string lane_type = "";
  double path_narrowest_width = 0;

  // Dedicated Debug Logging
  std::ofstream debug_log;
  debug_log.open("/apollo/data/log/lane_follow_debug.log", std::ios::app);
  if (debug_log.is_open()) {
      debug_log << "\n[Time: " << apollo::cyber::Clock::NowInSeconds() << "] DecidePathBounds Start" << std::endl;
      
      // 1. Analyze PathDecision Obstacles (Pre-filter)
      auto obstacles = reference_line_info_->path_decision()->obstacles();
      debug_log << "PathDecision Total Obstacles: " << obstacles.Items().size() << std::endl;
      for (const auto* obs : obstacles.Items()) {
          debug_log << "  > Obs ID: " << obs->Id() 
                    << " Static: " << obs->IsStatic()
                    << " Virtual: " << obs->IsVirtual()
                    << " Blocking: " << obs->IsBlockingObstacle()
                    << " LateralDecision: " << obs->LateralDecision().DebugString()
                    << " SL: [" << obs->PerceptionSLBoundary().start_s() << ", " << obs->PerceptionSLBoundary().end_s() 
                    << "] x [" << obs->PerceptionSLBoundary().start_l() << ", " << obs->PerceptionSLBoundary().end_l() << "]"
                    << std::endl;
      }
  }

  // 1. Initialize the path boundaries to be an indefinitely large area.
  if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                               &path_bound, init_sl_state_)) {
    const std::string msg = "Failed to initialize path boundaries.";
    AERROR << msg;
    if (debug_log.is_open()) debug_log << "InitPathBoundary Failed" << std::endl; 
    return false;
  }
  
  std::string borrow_lane_type;
  bool is_include_adc = config_.is_extend_lane_bounds_to_include_adc() &&
                        !injector_->planning_context()
                             ->planning_status()
                             .path_decider()
                             .is_in_path_lane_borrow_scenario();
  // 2. Decide a rough boundary based on lane info and ADC's position
  if (!PathBoundsDeciderUtil::GetBoundaryFromSelfLane(
          *reference_line_info_, init_sl_state_, &path_bound)) {
    AERROR << "Failed to decide a rough boundary based on self lane.";
    return false;
  }
  if (is_include_adc) {
    PathBoundsDeciderUtil::ExtendBoundaryByADC(
        *reference_line_info_, init_sl_state_, config_.extend_buffer(),
        &path_bound);
  }
  
  // Print curves logic...
  PrintCurves print_curve;
  auto indexed_obstacles = reference_line_info_->path_decision()->obstacles();
  for (const auto* obs : indexed_obstacles.Items()) {
    const auto& sl_bound = obs->PerceptionSLBoundary();
    for (int i = 0; i < sl_bound.boundary_point_size(); i++) {
      std::string name = obs->Id() + "_obs_sl_boundary";
      print_curve.AddPoint(name, sl_bound.boundary_point(i).s(),
                           sl_bound.boundary_point(i).l());
    }
  }
  print_curve.PrintToLog();

  path_bound.set_label(absl::StrCat("regular/", "self"));

  // 3. Fine-tune the boundary based on static obstacles
  PathBound temp_path_bound = path_bound;
  std::vector<SLPolygon> obs_sl_polygons;
  PathBoundsDeciderUtil::GetSLPolygons(*reference_line_info_, &obs_sl_polygons,
                                       init_sl_state_);

  // Debug Log GetSLPolygons Result
  if (debug_log.is_open()) {
      debug_log << "GetSLPolygons Result Count: " << obs_sl_polygons.size() << std::endl;
      for (const auto& p : obs_sl_polygons) {
          debug_log << "  > Poly ID: " << p.id() << " NudgeInfo: " << p.NudgeInfo() << std::endl;
      }
  }

  if (!PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles(
          *reference_line_info_, &obs_sl_polygons, init_sl_state_, &path_bound,
          &blocking_obstacle_id, &path_narrowest_width)) {
    const std::string msg =
        "Failed to decide fine tune the boundaries after "
        "taking into consideration all static obstacles.";
    AERROR << msg;
    if (debug_log.is_open()) debug_log << "GetBoundaryFromStaticObstacles Failed" << std::endl;
    return false;
  }

  // --- Nudge Conflict Detection Logic ---
  if (debug_log.is_open()) {
      debug_log << "Initial blocking_obstacle_id: [" << blocking_obstacle_id << "]" << std::endl;
      debug_log << "Path narrowest width: " << path_narrowest_width << std::endl;
  }

  if (blocking_obstacle_id.empty()) {
    bool has_left = false, has_right = false;
    std::string left_id, right_id;
    int lc = 0, rc = 0, bc = 0, ic = 0;
    
    for (const auto& p : obs_sl_polygons) {
      auto n = p.NudgeInfo();
      if (n == SLPolygon::LEFT_NUDGE) {
        has_left = true; left_id = p.id(); lc++;
      } else if (n == SLPolygon::RIGHT_NUDGE) {
        has_right = true; right_id = p.id(); rc++;
      } else if (n == SLPolygon::BLOCKED) {
        bc++;
      } else if (n == SLPolygon::IGNORE) {
        ic++;
      }
    }
    
    if (debug_log.is_open()) {
        debug_log << "Nudge Summary: L=" << lc << ", R=" << rc << ", B=" << bc << ", I=" << ic << std::endl;
    }
    
    if (has_left && has_right) {
      double w = VehicleConfigHelper::GetConfig().vehicle_param().width();
      double m = w + 0.5;
      
      if (debug_log.is_open()) {
          debug_log << "CONFLICT DETECTED! Left+Right Nudge." << std::endl;
          debug_log << "Vehicle Width: " << w << ", Min Passable: " << m << ", Actual: " << path_narrowest_width << std::endl;
      }
      
      if (path_narrowest_width < m) {
        blocking_obstacle_id = right_id.empty() ? left_id : right_id;
        if (debug_log.is_open()) debug_log << ">>> SETTING BLOCKING ID: " << blocking_obstacle_id << " <<<" << std::endl;
        AINFO << "[LaneFollowPath] CONFLICT BLOCKED! Set id: " << blocking_obstacle_id;
      } else {
        if (debug_log.is_open()) debug_log << "Width sufficient, PASSABLE." << std::endl;
      }
    }
  }

  // Close log file
  if (debug_log.is_open()) debug_log.close();

  // 4. Append some extra path bound points to avoid zero-length path data.
  int counter = 0;
  while (!blocking_obstacle_id.empty() &&
         path_bound.size() < temp_path_bound.size() &&
         counter < FLAGS_num_extra_tail_bound_point) {
    path_bound.push_back(temp_path_bound[path_bound.size()]);
    counter++;
  }

  // lane_follow_status update
  auto* lane_follow_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_lane_follow();
  if (!blocking_obstacle_id.empty()) {
    double current_time = ::apollo::cyber::Clock::NowInSeconds();
    lane_follow_status->set_block_obstacle_id(blocking_obstacle_id);
    if (lane_follow_status->lane_follow_block()) {
      lane_follow_status->set_block_duration(
          lane_follow_status->block_duration() + current_time -
          lane_follow_status->last_block_timestamp());
    } else {
      lane_follow_status->set_block_duration(0);
      lane_follow_status->set_lane_follow_block(true);
    }
    lane_follow_status->set_last_block_timestamp(current_time);
  } else {
    if (lane_follow_status->lane_follow_block()) {
      lane_follow_status->set_block_duration(0);
      lane_follow_status->set_lane_follow_block(false);
      lane_follow_status->set_last_block_timestamp(0);
    }
  }

  ADEBUG << "Completed generating path boundaries.";
  if (init_sl_state_.second[0] > path_bound[0].l_upper.l ||
      init_sl_state_.second[0] < path_bound[0].l_lower.l) {
    AINFO << "not in self lane maybe lane borrow or out of road. init l : "
          << init_sl_state_.second[0] << ", path_bound l: [ "
          << path_bound[0].l_lower.l << "," << path_bound[0].l_upper.l << " ]";
    return false;
  }

  path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
  RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
  return true;
}

bool LaneFollowPath::OptimizePath(
    const std::vector<PathBoundary>& path_boundaries,
    std::vector<PathData>* candidate_path_data) {
  const auto& config = config_.path_optimizer_config();
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};
  for (const auto& path_boundary : path_boundaries) {
    size_t path_boundary_size = path_boundary.boundary().size();
    if (path_boundary_size <= 1U) {
      AERROR << "Get invalid path boundary with size: " << path_boundary_size;
      return false;
    }
    std::vector<double> opt_l, opt_dl, opt_ddl;
    std::vector<std::pair<double, double>> ddl_bounds;
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line,
                                         &ddl_bounds);
    PrintCurves print_debug;
    for (size_t i = 0; i < path_boundary_size; ++i) {
      double s = static_cast<double>(i) * path_boundary.delta_s() +
                 path_boundary.start_s();
      double kappa = reference_line.GetNearestReferencePoint(s).kappa();
      print_debug.AddPoint("ref_kappa", s, kappa);
    }
    print_debug.PrintToLog();
    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(
        std::fmax(init_sl_state_.first[1], 1e-12));
    std::vector<double> ref_l(path_boundary_size, 0);
    std::vector<double> weight_ref_l(path_boundary_size, 0);

    PathOptimizerUtil::UpdatePathRefWithBound(
        path_boundary, config.path_reference_l_weight(), &ref_l, &weight_ref_l);
    bool res_opt = PathOptimizerUtil::OptimizePath(
        init_sl_state_, end_state, ref_l, weight_ref_l, path_boundary,
        ddl_bounds, jerk_bound, config, &opt_l, &opt_dl, &opt_ddl);
    if (res_opt) {
      auto frenet_frame_path = PathOptimizerUtil::ToPiecewiseJerkPath(
          opt_l, opt_dl, opt_ddl, path_boundary.delta_s(),
          path_boundary.start_s());
      PathData path_data;
      path_data.SetReferenceLine(&reference_line);
      path_data.SetFrenetPath(std::move(frenet_frame_path));
      if (FLAGS_use_front_axe_center_in_path_planning) {
        auto discretized_path = DiscretizedPath(
            PathOptimizerUtil::ConvertPathPointRefFromFrontAxeToRearAxe(
                path_data));
        path_data.SetDiscretizedPath(discretized_path);
      }
      path_data.set_path_label(path_boundary.label());
      path_data.set_blocking_obstacle_id(path_boundary.blocking_obstacle_id());
      candidate_path_data->push_back(std::move(path_data));
      PrintCurves print_path_kappa;
      for (const auto& p : candidate_path_data->back().discretized_path()) {
        print_path_kappa.AddPoint(path_boundary.label() + "_path_kappa",
                                  p.s() + init_sl_state_.first[0], p.kappa());
      }
      print_path_kappa.PrintToLog();
    }
  }
  if (candidate_path_data->empty()) {
    return false;
  }
  return true;
}

bool LaneFollowPath::AssessPath(std::vector<PathData>* candidate_path_data,
                                PathData* final_path) {
  PathData& curr_path_data = candidate_path_data->back();
  RecordDebugInfo(curr_path_data, curr_path_data.path_label(),
                  reference_line_info_);
  if (!PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_,
                                                     curr_path_data)) {
    AINFO << "Lane follow path is invalid";
    return false;
  }

  std::vector<PathPointDecision> path_decision;
  PathAssessmentDeciderUtil::InitPathPointDecision(
      curr_path_data, PathData::PathPointType::IN_LANE, &path_decision);
  curr_path_data.SetPathPointDecisionGuide(std::move(path_decision));

  if (curr_path_data.Empty()) {
    AINFO << "Lane follow path is empty after trimed";
    return false;
  }
  *final_path = curr_path_data;
  AINFO << final_path->path_label() << final_path->blocking_obstacle_id();
  reference_line_info_->MutableCandidatePathData()->push_back(*final_path);
  reference_line_info_->SetBlockingObstacle(
      curr_path_data.blocking_obstacle_id());
  return true;
}

}  // namespace planning
}  // namespace apollo

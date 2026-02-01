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

#include "modules/planning/tasks/lane_borrow_path/lane_borrow_path.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <fstream>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/obstacle_blocking_analyzer.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_interface_base/task_base/common/path_generation.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_assessment_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_bounds_decider_util.h"
#include "modules/planning/planning_interface_base/task_base/common/path_util/path_optimizer_util.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

namespace compat {

// C++17 void_t polyfill for C++14 support
template <typename... Ts> struct make_void { typedef void type; };
template <typename... Ts> using void_t = typename make_void<Ts...>::type;

// SFINAE helper to detect if the 5-parameter version exists
template <typename T, typename = void>
struct has_5_param_func : std::false_type {};

template <typename T>
struct has_5_param_func<T, void_t<decltype(T::GetBoundaryFromStaticObstacles(
    std::declval<const apollo::planning::ReferenceLineInfo&>(),
    std::declval<const apollo::planning::SLState&>(),
    std::declval<apollo::planning::PathBoundary*>(),
    std::declval<std::string*>(),
    std::declval<double*>()
))>> : std::true_type {};

// Official Environment (5 params) - Enabled if has_5_param_func is true
template <typename T>
typename std::enable_if<has_5_param_func<T>::value, bool>::type
GetBoundary(
    const apollo::planning::ReferenceLineInfo& info, 
    const apollo::planning::SLState& state,
    apollo::planning::PathBoundary* bound, 
    std::string* block_id, 
    double* width) {
    return T::GetBoundaryFromStaticObstacles(info, state, bound, block_id, width);
}

// Local Environment (6 params) - Enabled if has_5_param_func is false
template <typename T>
typename std::enable_if<!has_5_param_func<T>::value, bool>::type
GetBoundary(
    const apollo::planning::ReferenceLineInfo& info, 
    const apollo::planning::SLState& state,
    apollo::planning::PathBoundary* bound, 
    std::string* block_id, 
    double* width) {
    std::vector<apollo::planning::SLPolygon> obs_polygons;
    
    // Copy state and cast info to non-const for local API compatibility
    apollo::planning::SLState state_copy = state; 
    apollo::planning::ReferenceLineInfo& info_ref = const_cast<apollo::planning::ReferenceLineInfo&>(info);
    
    T::GetSLPolygons(info_ref, &obs_polygons, state_copy);
    return T::GetBoundaryFromStaticObstacles(info_ref, &obs_polygons, state_copy, bound, block_id, width);
}

} // namespace compat

// Fallback for missing flags in official environment
#ifndef FLAGS_path_trim_destination_threshold
static const double FLAGS_path_trim_destination_threshold = 15.0;
#endif

constexpr double kIntersectionClearanceDist = 20.0;
constexpr double kJunctionClearanceDist = 15.0;

// Global debug log file
static std::ofstream g_lane_borrow_log;

void WriteLaneBorrowDebug(const std::string& msg) {
  if (!g_lane_borrow_log.is_open()) {
    g_lane_borrow_log.open("/apollo/data/log/lane_borrow_debug.log", std::ios::app);
  }
  if (g_lane_borrow_log.is_open()) {
    g_lane_borrow_log << apollo::cyber::Clock::NowInSeconds() << " " << msg << std::endl;
    g_lane_borrow_log.flush();
  }
}

common::Status LaneBorrowPath::Init(const LaneBorrowPathConfig& config) {
  config_ = config;
  AINFO << "[LaneBorrowPath] Init called with enable_debug_log: " << config_.enable_debug_log();
  return common::Status::OK();
}

bool LaneBorrowPath::Init(const std::string& config_dir,
                          const std::string& name,
                          const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  return Task::LoadConfig<LaneBorrowPathConfig>(&config_);
}

apollo::common::Status LaneBorrowPath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  
  WriteLaneBorrowDebug("\n=== LaneBorrowPath::Process ===");
  WriteLaneBorrowDebug("ADC speed: " + std::to_string(frame->PlanningStartPoint().v()));
  
  if (!config_.is_allow_lane_borrowing() ||
      reference_line_info->path_reusable()) {
    WriteLaneBorrowDebug("Skipping: allow_borrow=" + std::to_string(config_.is_allow_lane_borrowing()) 
                         + ", reusable=" + std::to_string(reference_line_info->path_reusable()));
    return Status::OK();
  }
  if (!IsNecessaryToBorrowLane()) {
    WriteLaneBorrowDebug("IsNecessaryToBorrowLane returned false.");
    return Status::OK();
  }
  
  WriteLaneBorrowDebug("*** LANE BORROW ACTIVE ***");
  WriteLaneBorrowDebug("decided_side_pass_direction count: " + std::to_string(decided_side_pass_direction_.size()));
  for (size_t i = 0; i < decided_side_pass_direction_.size(); i++) {
    std::string dir = (decided_side_pass_direction_[i] == SidePassDirection::LEFT_BORROW) ? "LEFT" : "RIGHT";
    WriteLaneBorrowDebug("  Direction[" + std::to_string(i) + "]: " + dir);
  }
  
  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<PathData> candidate_path_data;

  GetStartPointSLState();
  if (!DecidePathBounds(&candidate_path_boundaries)) {
    WriteLaneBorrowDebug("DecidePathBounds FAILED");
    return Status::OK();
  }
  
  WriteLaneBorrowDebug("DecidePathBounds returned " + std::to_string(candidate_path_boundaries.size()) + " boundaries");
  
  if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
    WriteLaneBorrowDebug("OptimizePath FAILED");
    return Status::OK();
  }
  
  WriteLaneBorrowDebug("OptimizePath returned " + std::to_string(candidate_path_data.size()) + " paths");
  
  if (AssessPath(&candidate_path_data,
                 reference_line_info->mutable_path_data())) {
    WriteLaneBorrowDebug("AssessPath SUCCESS - lane borrow path generated");
  } else {
    WriteLaneBorrowDebug("AssessPath FAILED");
  }

  return Status::OK();
}

bool LaneBorrowPath::DecidePathBounds(std::vector<PathBoundary>* boundary) {
  for (size_t i = 0; i < decided_side_pass_direction_.size(); i++) {
    std::string direction = (decided_side_pass_direction_[i] == SidePassDirection::LEFT_BORROW) ? "LEFT" : "RIGHT";
    WriteLaneBorrowDebug("\n--- DecidePathBounds [" + direction + "] ---");
    
    boundary->emplace_back();
    auto& path_bound = boundary->back();
    std::string blocking_obstacle_id = "";
    std::string borrow_lane_type = "";
    double path_narrowest_width = 0;
    
    // 1. Initialize
    if (!PathBoundsDeciderUtil::InitPathBoundary(*reference_line_info_,
                                                 &path_bound, init_sl_state_)) {
      WriteLaneBorrowDebug("InitPathBoundary FAILED");
      boundary->pop_back();
      continue;
    }
    WriteLaneBorrowDebug("InitPathBoundary OK, size=" + std::to_string(path_bound.size()));
    
    // 2. Get neighbor lane boundary
    if (!GetBoundaryFromNeighborLane(decided_side_pass_direction_[i],
                                     &path_bound, &borrow_lane_type)) {
      WriteLaneBorrowDebug("GetBoundaryFromNeighborLane FAILED");
      boundary->pop_back();
      continue;
    }
    WriteLaneBorrowDebug("GetBoundaryFromNeighborLane OK, type=" + borrow_lane_type);

    std::string label = (decided_side_pass_direction_[i] == SidePassDirection::LEFT_BORROW) 
                          ? ("regular/left" + borrow_lane_type) 
                          : ("regular/right" + borrow_lane_type);
    path_bound.set_label(label);
    WriteLaneBorrowDebug("Path label: " + label);

    // 3. Static obstacles
    PathBound temp_path_bound = path_bound;
    
    // DEBUG: Log PathDecision obstacles
    auto obstacles = reference_line_info_->path_decision()->obstacles();
    WriteLaneBorrowDebug("PathDecision Total Obstacles: " + std::to_string(obstacles.Items().size()));
    for (const auto* obs : obstacles.Items()) {
        WriteLaneBorrowDebug("  > Obs ID: " + obs->Id() + 
                             " Static: " + std::to_string(obs->IsStatic()) +
                             " SL: [" + std::to_string(obs->PerceptionSLBoundary().start_s()) + "," + std::to_string(obs->PerceptionSLBoundary().end_s()) + "]" +
                             " x [" + std::to_string(obs->PerceptionSLBoundary().start_l()) + "," + std::to_string(obs->PerceptionSLBoundary().end_l()) + "]");
    }

    if (!compat::GetBoundary<PathBoundsDeciderUtil>(
            *reference_line_info_, init_sl_state_,
            &path_bound, &blocking_obstacle_id, &path_narrowest_width)) {
      WriteLaneBorrowDebug("GetBoundaryFromStaticObstacles FAILED");
      boundary->pop_back();
      continue;
    }
    WriteLaneBorrowDebug("GetBoundaryFromStaticObstacles OK");
    WriteLaneBorrowDebug("  blocking_obstacle_id: [" + blocking_obstacle_id + "]");
    WriteLaneBorrowDebug("  path_narrowest_width: " + std::to_string(path_narrowest_width));
    WriteLaneBorrowDebug("  path_bound size after obs: " + std::to_string(path_bound.size()));

    // 4. Extra points
    int counter = 0;
    while (!blocking_obstacle_id.empty() &&
           path_bound.size() < temp_path_bound.size() &&
           counter < FLAGS_num_extra_tail_bound_point) {
      path_bound.push_back(temp_path_bound[path_bound.size()]);
      counter++;
    }

    path_bound.set_blocking_obstacle_id(blocking_obstacle_id);
    RecordDebugInfo(path_bound, path_bound.label(), reference_line_info_);
  }
  return !boundary->empty();
}

bool LaneBorrowPath::OptimizePath(
    const std::vector<PathBoundary>& path_boundaries,
    std::vector<PathData>* candidate_path_data) {
  const auto& config = config_.path_optimizer_config();
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  std::array<double, 3> end_state = {0.0, 0.0, 0.0};

  for (const auto& path_boundary : path_boundaries) {
    std::vector<double> opt_l, opt_dl, opt_ddl;
    std::vector<std::pair<double, double>> ddl_bounds;
    PathOptimizerUtil::CalculateAccBound(path_boundary, reference_line,
                                         &ddl_bounds);
    const double jerk_bound = PathOptimizerUtil::EstimateJerkBoundary(
        std::fmax(init_sl_state_.first[1], 1e-12));
    std::vector<double> ref_l;
    std::vector<double> weight_ref_l;
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
    }
  }
  if (candidate_path_data->empty()) {
    return false;
  }
  return true;
}

bool LaneBorrowPath::AssessPath(std::vector<PathData>* candidate_path_data,
                                PathData* final_path) {
  std::vector<PathData> valid_path_data;
  for (auto& curr_path_data : *candidate_path_data) {
    if (PathAssessmentDeciderUtil::IsValidRegularPath(*reference_line_info_,
                                                      curr_path_data)) {
      SetPathInfo(&curr_path_data);
      if (reference_line_info_->SDistanceToDestination() <
          FLAGS_path_trim_destination_threshold) {
        PathAssessmentDeciderUtil::TrimTailingOutLanePoints(&curr_path_data);
      }
      if (curr_path_data.Empty()) {
        WriteLaneBorrowDebug("Path empty after trim: " + curr_path_data.path_label());
        continue;
      }
      valid_path_data.push_back(curr_path_data);
    } else {
      WriteLaneBorrowDebug("Path invalid: " + curr_path_data.path_label());
    }
  }
  if (valid_path_data.empty()) {
    WriteLaneBorrowDebug("All lane borrow paths are invalid");
    return false;
  }
  
  WriteLaneBorrowDebug("Valid paths: " + std::to_string(valid_path_data.size()));
  
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  const std::string blocking_obstacle_id =
      mutable_path_decider_status->front_static_obstacle_id();
  const Obstacle* blocking_obstacle =
      reference_line_info_->path_decision()->obstacles().Find(
          blocking_obstacle_id);
  if (valid_path_data.size() > 1) {
    if (ComparePathData(valid_path_data[0], valid_path_data[1],
                        blocking_obstacle)) {
      *final_path = valid_path_data[0];
    } else {
      *final_path = valid_path_data[1];
    }
  } else {
    *final_path = valid_path_data[0];
  }
  
  WriteLaneBorrowDebug("Selected path: " + final_path->path_label());
  
  RecordDebugInfo(*final_path, final_path->path_label(), reference_line_info_);
  return true;
}

bool LaneBorrowPath::GetBoundaryFromNeighborLane(
    const SidePassDirection pass_direction, PathBoundary* const path_bound,
    std::string* borrow_lane_type) {
  CHECK_NOTNULL(path_bound);
  ACHECK(!path_bound->empty());
  const ReferenceLine& reference_line = reference_line_info_->reference_line();
  double adc_lane_width = PathBoundsDeciderUtil::GetADCLaneWidth(
      reference_line, init_sl_state_.first[0]);
  double offset_to_map = 0;
  bool borrowing_reverse_lane = false;
  reference_line.GetOffsetToMap(init_sl_state_.first[0], &offset_to_map);
  
  std::string dir_str = (pass_direction == SidePassDirection::LEFT_BORROW) ? "LEFT" : "RIGHT";
  WriteLaneBorrowDebug("GetBoundaryFromNeighborLane [" + dir_str + "]");
  WriteLaneBorrowDebug("  ADC lane width: " + std::to_string(adc_lane_width));
  WriteLaneBorrowDebug("  Offset to map: " + std::to_string(offset_to_map));
  
  double past_lane_left_width = adc_lane_width / 2.0;
  double past_lane_right_width = adc_lane_width / 2.0;
  int path_blocked_idx = -1;
  
  // NEW: Find the NEAREST obstacle's L value for dynamic left boundary adjustment
  double nearest_obstacle_l = 0.0;  // Default: reference line center
  if (pass_direction == SidePassDirection::RIGHT_BORROW) {
    auto obstacles = reference_line_info_->path_decision()->obstacles();
    std::vector<const Obstacle*> static_obstacles;
    for (const auto* obs : obstacles.Items()) {
      if (obs->IsStatic() && !obs->IsVirtual() && 
          obs->PerceptionSLBoundary().start_s() > 0.0) {
        static_obstacles.push_back(obs);
      }
    }
    
    if (!static_obstacles.empty()) {
      // Sort by start_s (proximity)
      std::sort(static_obstacles.begin(), static_obstacles.end(), 
                [](const Obstacle* a, const Obstacle* b) {
                  return a->PerceptionSLBoundary().start_s() < b->PerceptionSLBoundary().start_s();
                });
      
      const auto* nearest_obs = static_obstacles.front();
      nearest_obstacle_l = nearest_obs->PerceptionSLBoundary().end_l();
      WriteLaneBorrowDebug("  Nearest valid static obstacle [" + nearest_obs->Id() 
                           + "] s=" + std::to_string(nearest_obs->PerceptionSLBoundary().start_s())
                           + " L (for left boundary): " + std::to_string(nearest_obstacle_l));
    }
  }
  
  // Log first few points for debugging
  int log_count = 0;
  const int max_log = 5;
  
  for (size_t i = 0; i < path_bound->size(); ++i) {
    double curr_s = (*path_bound)[i].s;
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    double offset_to_lane_center = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, &curr_lane_left_width,
                                     &curr_lane_right_width)) {
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    } else {
      reference_line.GetOffsetToMap(curr_s, &offset_to_lane_center);
      curr_lane_left_width += offset_to_lane_center;
      curr_lane_right_width -= offset_to_lane_center;
      past_lane_left_width = curr_lane_left_width;
      past_lane_right_width = curr_lane_right_width;
    }
    
    double curr_neighbor_lane_width = 0.0;
    hdmap::Id neighbor_lane_id;
    std::string neighbor_info = "NONE";
    
    if (CheckLaneBoundaryType(*reference_line_info_, curr_s, pass_direction)) {
      if (pass_direction == SidePassDirection::LEFT_BORROW) {
        if (reference_line_info_->GetNeighborLaneInfo(
                ReferenceLineInfo::LaneType::LeftForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          neighbor_info = "LeftForward: " + neighbor_lane_id.id() + " w=" + std::to_string(curr_neighbor_lane_width);
        } else if (reference_line_info_->GetNeighborLaneInfo(
                       ReferenceLineInfo::LaneType::LeftReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          neighbor_info = "LeftReverse: " + neighbor_lane_id.id() + " w=" + std::to_string(curr_neighbor_lane_width);
        }
      } else {
        if (reference_line_info_->GetNeighborLaneInfo(
                ReferenceLineInfo::LaneType::RightForward, curr_s,
                &neighbor_lane_id, &curr_neighbor_lane_width)) {
          neighbor_info = "RightForward: " + neighbor_lane_id.id() + " w=" + std::to_string(curr_neighbor_lane_width);
        } else if (reference_line_info_->GetNeighborLaneInfo(
                       ReferenceLineInfo::LaneType::RightReverse, curr_s,
                       &neighbor_lane_id, &curr_neighbor_lane_width)) {
          borrowing_reverse_lane = true;
          neighbor_info = "RightReverse: " + neighbor_lane_id.id() + " w=" + std::to_string(curr_neighbor_lane_width);
        }
      }
    }
    
    double curr_left_bound_lane =
        curr_lane_left_width + (pass_direction == SidePassDirection::LEFT_BORROW
                                    ? (curr_neighbor_lane_width + 6.0)
                                    : 0.0);
    // NEW: For RIGHT_BORROW, shift left boundary right to match nearest obstacle
    if (pass_direction == SidePassDirection::RIGHT_BORROW && nearest_obstacle_l != 0.0) {
      // Use the nearest obstacle's L value as left boundary (with small buffer)
      curr_left_bound_lane = nearest_obstacle_l + 0.1;  // 0.1m safety buffer
    }
    double curr_right_bound_lane =
        -curr_lane_right_width -
        (pass_direction == SidePassDirection::RIGHT_BORROW
             ? (curr_neighbor_lane_width + 6.0)
             : 0.0);
    double curr_left_bound = curr_left_bound_lane - offset_to_map;
    double curr_right_bound = curr_right_bound_lane - offset_to_map;

    // Log first few points
    if (log_count < max_log) {
      WriteLaneBorrowDebug("  Point[" + std::to_string(i) + "] s=" + std::to_string(curr_s) 
                           + " LaneL=" + std::to_string(curr_lane_left_width)
                           + " LaneR=" + std::to_string(curr_lane_right_width)
                           + " Neighbor=" + neighbor_info
                           + " BoundL=" + std::to_string(curr_left_bound)
                           + " BoundR=" + std::to_string(curr_right_bound));
      log_count++;
    }

    if (!PathBoundsDeciderUtil::UpdatePathBoundaryWithBuffer(
            curr_left_bound, curr_right_bound, BoundType::LANE, BoundType::LANE,
            "", "", &path_bound->at(i))) {
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      WriteLaneBorrowDebug("  Path blocked at index: " + std::to_string(path_blocked_idx));
      break;
    }
  }
  PathBoundsDeciderUtil::TrimPathBounds(path_blocked_idx, path_bound);
  *borrow_lane_type = borrowing_reverse_lane ? "reverse" : "forward";
  
  WriteLaneBorrowDebug("  Final path_bound size: " + std::to_string(path_bound->size()));
  WriteLaneBorrowDebug("  Borrow lane type: " + *borrow_lane_type);
  
  return true;
}

void LaneBorrowPath::UpdateSelfPathInfo() {
  auto cur_path = reference_line_info_->path_data();
  if (!cur_path.Empty() &&
      cur_path.path_label().find("self") != std::string::npos &&
      cur_path.blocking_obstacle_id().empty()) {
    use_self_lane_ = std::min(use_self_lane_ + 1, 10);
  } else {
    use_self_lane_ = 0;
  }
  blocking_obstacle_id_ = cur_path.blocking_obstacle_id();
}

bool LaneBorrowPath::IsNecessaryToBorrowLane() {
  auto* mutable_path_decider_status = injector_->planning_context()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  WriteLaneBorrowDebug("Checking IsNecessaryToBorrowLane...");
  
  if (mutable_path_decider_status->is_in_path_lane_borrow_scenario()) {
    UpdateSelfPathInfo();
    if (use_self_lane_ >= 6) {
      mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
      decided_side_pass_direction_.clear();
      WriteLaneBorrowDebug("Switching from LANE-BORROW to SELF-LANE (use_self_lane=" + std::to_string(use_self_lane_) + ")");
    }
  } else {
    std::string blocking_id = mutable_path_decider_status->front_static_obstacle_id();
    WriteLaneBorrowDebug("Not in lane borrow. Blocking obstacle: [" + blocking_id + "]");
    
    if (!HasSingleReferenceLine(*frame_)) {
      WriteLaneBorrowDebug("FAIL: Multiple reference lines (" + std::to_string(frame_->reference_line_info().size()) + ")");
      return false;
    }
    if (!IsWithinSidePassingSpeedADC(*frame_)) {
      WriteLaneBorrowDebug("FAIL: Speed too high (" + std::to_string(frame_->PlanningStartPoint().v()) + " > " + std::to_string(config_.lane_borrow_max_speed()) + ")");
      return false;
    }
    if (!IsBlockingObstacleFarFromIntersection(*reference_line_info_)) {
      WriteLaneBorrowDebug("FAIL: Obstacle too close to intersection");
      return false;
    }
    if (!IsLongTermBlockingObstacle()) {
      int cycle = injector_->planning_context()->planning_status().path_decider().front_static_obstacle_cycle_counter();
      WriteLaneBorrowDebug("FAIL: Not long-term blocking (counter=" + std::to_string(cycle) + ")");
      return false;
    }
    if (!IsBlockingObstacleWithinDestination(*reference_line_info_)) {
      WriteLaneBorrowDebug("FAIL: Blocking obstacle beyond destination");
      return false;
    }
    if (!IsSidePassableObstacle(*reference_line_info_)) {
      WriteLaneBorrowDebug("FAIL: Obstacle not side-passable");
      return false;
    }

    if (decided_side_pass_direction_.empty()) {
      bool left_borrowable;
      bool right_borrowable;
      CheckLaneBorrow(*reference_line_info_, &left_borrowable, &right_borrowable);
      WriteLaneBorrowDebug("CheckLaneBorrow: left=" + std::to_string(left_borrowable) + ", right=" + std::to_string(right_borrowable));
      
      if (!left_borrowable && !right_borrowable) {
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(false);
        WriteLaneBorrowDebug("FAIL: Neither lane borrowable");
        return false;
      } else {
        mutable_path_decider_status->set_is_in_path_lane_borrow_scenario(true);
        if (left_borrowable) {
          decided_side_pass_direction_.push_back(SidePassDirection::LEFT_BORROW);
        }
        if (right_borrowable) {
          decided_side_pass_direction_.push_back(SidePassDirection::RIGHT_BORROW);
        }
      }
    }
    use_self_lane_ = 0;
    WriteLaneBorrowDebug("SUCCESS: Switching to LANE-BORROW mode");
  }
  return mutable_path_decider_status->is_in_path_lane_borrow_scenario();
}

bool LaneBorrowPath::HasSingleReferenceLine(const Frame& frame) {
  return frame.reference_line_info().size() == 1;
}

bool LaneBorrowPath::IsWithinSidePassingSpeedADC(const Frame& frame) {
  return frame.PlanningStartPoint().v() < config_.lane_borrow_max_speed();
}

bool LaneBorrowPath::IsLongTermBlockingObstacle() {
  if (injector_->planning_context()
          ->planning_status()
          .path_decider()
          .front_static_obstacle_cycle_counter() >=
      config_.long_term_blocking_obstacle_cycle_threshold()) {
    return true;
  }
  return false;
}

bool LaneBorrowPath::IsBlockingObstacleWithinDestination(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    return true;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    return true;
  }

  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().start_s();
  double adc_end_s = reference_line_info.AdcSlBoundary().end_s();
  if (blocking_obstacle_s - adc_end_s >
      reference_line_info.SDistanceToDestination()) {
    return false;
  }
  return true;
}

bool LaneBorrowPath::IsBlockingObstacleFarFromIntersection(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    return true;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    return true;
  }

  double blocking_obstacle_s =
      blocking_obstacle->PerceptionSLBoundary().end_s();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first != ReferenceLineInfo::SIGNAL &&
        overlap.first != ReferenceLineInfo::STOP_SIGN) {
      continue;
    }
    auto distance = overlap.second.start_s - blocking_obstacle_s;
    if (overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::STOP_SIGN) {
      if (distance < kIntersectionClearanceDist) {
        return false;
      }
    } else {
      if (distance < kJunctionClearanceDist) {
        return false;
      }
    }
  }
  return true;
}

bool LaneBorrowPath::IsSidePassableObstacle(
    const ReferenceLineInfo& reference_line_info) {
  const auto& path_decider_status =
      injector_->planning_context()->planning_status().path_decider();
  const std::string blocking_obstacle_id =
      path_decider_status.front_static_obstacle_id();
  if (blocking_obstacle_id.empty()) {
    return false;
  }
  const Obstacle* blocking_obstacle =
      reference_line_info.path_decision().obstacles().Find(
          blocking_obstacle_id);
  if (blocking_obstacle == nullptr) {
    return false;
  }
  return IsNonmovableObstacle(reference_line_info, *blocking_obstacle);
}

void LaneBorrowPath::CheckLaneBorrow(
    const ReferenceLineInfo& reference_line_info,
    bool* left_neighbor_lane_borrowable, bool* right_neighbor_lane_borrowable) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();

  *left_neighbor_lane_borrowable = true;
  *right_neighbor_lane_borrowable = true;

  static constexpr double kLookforwardDistance = 100.0;
  double check_s = reference_line_info.AdcSlBoundary().end_s();
  const double lookforward_distance =
      std::min(check_s + kLookforwardDistance, reference_line.Length());
  while (check_s < lookforward_distance) {
    auto ref_point = reference_line.GetNearestReferencePoint(check_s);
    if (ref_point.lane_waypoints().empty()) {
      *left_neighbor_lane_borrowable = false;
      *right_neighbor_lane_borrowable = false;
      return;
    }
    auto ptr_lane_info = reference_line_info.LocateLaneInfo(check_s);
    if (ptr_lane_info->lane().left_neighbor_forward_lane_id().empty() &&
        ptr_lane_info->lane().left_neighbor_reverse_lane_id().empty()) {
      *left_neighbor_lane_borrowable = false;
    }
    if (ptr_lane_info->lane().right_neighbor_forward_lane_id().empty() &&
        ptr_lane_info->lane().right_neighbor_reverse_lane_id().empty()) {
      *right_neighbor_lane_borrowable = false;
    }
    const auto waypoint = ref_point.lane_waypoints().front();
    hdmap::LaneBoundaryType::Type lane_boundary_type =
        hdmap::LaneBoundaryType::UNKNOWN;

    if (*left_neighbor_lane_borrowable) {
      lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
      if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
          lane_boundary_type == hdmap::LaneBoundaryType::DOUBLE_YELLOW ||
          lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
        *left_neighbor_lane_borrowable = false;
      }
    }
    if (*right_neighbor_lane_borrowable) {
      lane_boundary_type = hdmap::RightBoundaryType(waypoint);
      if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
          lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
        *right_neighbor_lane_borrowable = false;
      }
    }
    check_s += 2.0;
  }
}

bool LaneBorrowPath::CheckLaneBoundaryType(
    const ReferenceLineInfo& reference_line_info, const double check_s,
    const SidePassDirection& lane_borrow_info) {
  const ReferenceLine& reference_line = reference_line_info.reference_line();
  auto ref_point = reference_line.GetNearestReferencePoint(check_s);
  if (ref_point.lane_waypoints().empty()) {
    return false;
  }

  const auto waypoint = ref_point.lane_waypoints().front();
  hdmap::LaneBoundaryType::Type lane_boundary_type =
      hdmap::LaneBoundaryType::UNKNOWN;
  if (lane_borrow_info == SidePassDirection::LEFT_BORROW) {
    lane_boundary_type = hdmap::LeftBoundaryType(waypoint);
  } else if (lane_borrow_info == SidePassDirection::RIGHT_BORROW) {
    lane_boundary_type = hdmap::RightBoundaryType(waypoint);
  }
  if (lane_boundary_type == hdmap::LaneBoundaryType::SOLID_YELLOW ||
      lane_boundary_type == hdmap::LaneBoundaryType::SOLID_WHITE) {
    return false;
  }
  return true;
}

void LaneBorrowPath::SetPathInfo(PathData* const path_data) {
  std::vector<PathPointDecision> path_decision;
  PathAssessmentDeciderUtil::InitPathPointDecision(
      *path_data, PathData::PathPointType::IN_LANE, &path_decision);
  const auto& discrete_path = path_data->discretized_path();
  bool is_prev_point_out_lane = false;
  SLBoundary ego_sl_boundary;
  for (size_t i = 0; i < discrete_path.size(); ++i) {
    if (!GetSLBoundary(*path_data, i, reference_line_info_, &ego_sl_boundary)) {
      continue;
    }
    double lane_left_width = 0.0;
    double lane_right_width = 0.0;
    double middle_s =
        (ego_sl_boundary.start_s() + ego_sl_boundary.end_s()) / 2.0;
    if (reference_line_info_->reference_line().GetLaneWidth(
            middle_s, &lane_left_width, &lane_right_width)) {
      double back_to_inlane_extra_buffer = 0.2;
      double in_and_out_lane_hysteresis_buffer =
          is_prev_point_out_lane ? back_to_inlane_extra_buffer : 0.0;
      if (ego_sl_boundary.end_l() >
              lane_left_width + in_and_out_lane_hysteresis_buffer ||
          ego_sl_boundary.start_l() <
              -lane_right_width - in_and_out_lane_hysteresis_buffer) {
        if (path_data->path_label().find("reverse") != std::string::npos) {
          std::get<1>((path_decision)[i]) =
              PathData::PathPointType::OUT_ON_REVERSE_LANE;
        } else if (path_data->path_label().find("forward") !=
                   std::string::npos) {
          std::get<1>((path_decision)[i]) =
              PathData::PathPointType::OUT_ON_FORWARD_LANE;
        } else {
          std::get<1>((path_decision)[i]) = PathData::PathPointType::UNKNOWN;
        }
        if (!is_prev_point_out_lane) {
          if (ego_sl_boundary.end_l() >
                  lane_left_width + back_to_inlane_extra_buffer ||
              ego_sl_boundary.start_l() <
                  -lane_right_width - back_to_inlane_extra_buffer) {
            is_prev_point_out_lane = true;
          }
        }
      } else {
        std::get<1>((path_decision)[i]) = PathData::PathPointType::IN_LANE;
        if (is_prev_point_out_lane) {
          is_prev_point_out_lane = false;
        }
      }
    } else {
      break;
    }
  }
  path_data->SetPathPointDecisionGuide(std::move(path_decision));
}

bool ComparePathData(const PathData& lhs, const PathData& rhs,
                     const Obstacle* blocking_obstacle) {
  static constexpr double kNeighborPathLengthComparisonTolerance = 25.0;
  double lhs_path_length = lhs.frenet_frame_path().back().s();
  double rhs_path_length = rhs.frenet_frame_path().back().s();
  if (std::fabs(lhs_path_length - rhs_path_length) >
      kNeighborPathLengthComparisonTolerance) {
    return lhs_path_length > rhs_path_length;
  }
  int lhs_on_reverse =
      ContainsOutOnReverseLane(lhs.path_point_decision_guide());
  int rhs_on_reverse =
      ContainsOutOnReverseLane(rhs.path_point_decision_guide());
  if (std::abs(lhs_on_reverse - rhs_on_reverse) > 6) {
    return lhs_on_reverse < rhs_on_reverse;
  }
  if (blocking_obstacle) {
    const double obstacle_l =
        (blocking_obstacle->PerceptionSLBoundary().start_l() +
         blocking_obstacle->PerceptionSLBoundary().end_l()) /
        2;
    return (obstacle_l > 0.0
                ? (lhs.path_label().find("right") != std::string::npos)
                : (lhs.path_label().find("left") != std::string::npos));
  } else {
    double adc_l = lhs.frenet_frame_path().front().l();
    if (adc_l < -1.0) {
      return lhs.path_label().find("right") != std::string::npos;
    } else if (adc_l > 1.0) {
      return lhs.path_label().find("left") != std::string::npos;
    }
  }
  static constexpr double kBackToSelfLaneComparisonTolerance = 20.0;
  int lhs_back_idx = GetBackToInLaneIndex(lhs.path_point_decision_guide());
  int rhs_back_idx = GetBackToInLaneIndex(rhs.path_point_decision_guide());
  double lhs_back_s = lhs.frenet_frame_path()[lhs_back_idx].s();
  double rhs_back_s = rhs.frenet_frame_path()[rhs_back_idx].s();
  if (std::fabs(lhs_back_s - rhs_back_s) > kBackToSelfLaneComparisonTolerance) {
    return lhs_back_idx < rhs_back_idx;
  }
  bool lhs_on_leftlane = lhs.path_label().find("left") != std::string::npos;
  return lhs_on_leftlane;
}

int ContainsOutOnReverseLane(
    const std::vector<PathPointDecision>& path_point_decision) {
  int ret = 0;
  for (const auto& curr_decision : path_point_decision) {
    if (std::get<1>(curr_decision) ==
        PathData::PathPointType::OUT_ON_REVERSE_LANE) {
      ++ret;
    }
  }
  return ret;
}

int GetBackToInLaneIndex(
    const std::vector<PathPointDecision>& path_point_decision) {
  for (int i = static_cast<int>(path_point_decision.size()) - 1; i >= 0; --i) {
    if (std::get<1>(path_point_decision[i]) !=
        PathData::PathPointType::IN_LANE) {
      return i;
    }
  }
  return 0;
}

}  // namespace planning
}  // namespace apollo

void apollo::planning::LaneBorrowPath::WriteDebugLog(const std::string& msg) {
  if (!config_.enable_debug_log()) return;
  std::ofstream log_file("/apollo/data/log/lane_borrow_debug.log", std::ios::app);
  if (log_file.is_open()) {
    log_file << apollo::cyber::Clock::NowInSeconds() << " [LaneBorrowPath] " << msg << std::endl;
    log_file.close();
  }
}

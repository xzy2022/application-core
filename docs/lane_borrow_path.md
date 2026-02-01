## 借道相关参数

1. 触发条件参数
这些参数决定了自动驾驶车辆在什么情况下会尝试“借道”（例如绕开前方违停车辆）：

is_allow_lane_borrowing (true):
- 作用： 总开关。如果设为 false，车辆将永远不会尝试借道避障，只会停车等待。

lane_borrow_max_speed (5.0):
- 作用： 限速阈值（单位：m/s）。
- 逻辑： 只有当车辆当前速度低于此值（5.0 m/s 约为 18 km/h）时，才会触发借道。这是为了安全考虑，避免在高速行驶时突然大跨度借道。

long_term_blocking_obstacle_cycle_threshold (3):
- 作用： 障碍物阻挡周期阈值。

**逻辑：**当前方障碍物连续出现在视野中并阻挡路径达到 3 个规划周期（Cycle）后，才会被认定为“长期阻挡障碍物”，从而触发借道逻辑。这可以防止因为传感器的瞬时闪烁或短暂停车而产生不必要的借道。

2. 路径优化权重 (path_optimizer_config)
借道路径是通过 Piecewise Jerk Path Optimizer 生成的。这些参数决定了生成的路径有多“顺滑”：

l_weight (1.0):
- 横向偏移 (L) 的权重。权重越高，车辆越倾向于贴近参考线（通常是车道中心线）。

dl_weight (20.0):
- 横向速度 (dL/dS) 的权重。限制路径的倾向角度，防止蛇形走位。

ddl_weight (1000.0):
- 横向加速度 (d2L/dS2) 的权重。影响路径的曲率。数值越大，路径转弯越缓。

dddl_weight (50000.0):
- 横向加加速度 (d3L/dS3, Jerk) 的权重。核心平滑度参数。数值越大，方向盘转动越平稳，避免突兀的转向。

path_reference_l_weight (100.0):
- 参考路径跟随权重。在借道过程中，会生成一个理想的避障参考线，该权重决定了实际路径对这条参考线的贴合程度。

lateral_derivative_bound_default (2.0):
- 横向导数边界。限制路径的最大横向偏移速率。

总结：它是如何发挥作用的？
判断阶段： LaneBorrowPath 任务首先检查 is_allow_lane_borrowing 是否开启，且速度是否低于 lane_borrow_max_speed。
分析阶段： 确认阻挡物是否超过 long_term_blocking_obstacle_cycle_threshold 周期，且判定为可绕过（Side Passable）。
生成阶段： 根据左/右借道边界，结合 path_optimizer_config 中的权重，计算出一条既能绕过障碍物、又符合车辆动力学（平滑）的路径供下游模块执行。


感知模块 → 障碍物检测 → PathBoundsDeciderUtil::GetBoundaryFromStaticObstacles 
→ blocking_obstacle_id → LANE_FOLLOW_PATH → SetBlockingObstacle 
→ PATH_DECIDER → front_static_obstacle_id → LANE_BORROW_PATH

ST_BOUNDARY_MAPPER（速度边界映射器）检测到了障碍物（6297、6474、8586 等），这是速度规划模块的一部分，它会让小车减速或停止，但不会触发借道。

## 借道模块关联


模块执行顺序：
LANE_BORROW_PATH - 检测是否需要借道，但因为没有识别到"阻塞障碍物"而返回 false
LANE_FOLLOW_PATH - 作为默认路径生成器执行（耗时 ~5-8ms）
PATH_DECIDER - 路径决策器，基于障碍物决定路径
SPEED_DECIDER - 速度决策器
st_boundary_mapper - 这是速度规划部分，检测到了障碍物 6297、6474、8586 等


障碍检测：
LANE_BORROW_PATH -> front_static_obstacle_id --> PATH_DECIDER --> reference_line_info->GetBlockingObstacle()

LANE_FOLLOW_PATH --> reference_line_info->GetBlockingObstacle() --> PATH_BOUNDS_DECIDER

真正的模块执行顺序
LANE_CHANGE_PATH → LANE_FOLLOW_PATH → LANE_BORROW_PATH → FALLBACK_PATH 
→ PATH_DECIDER → RULE_BASED_STOP_DECIDER → SPEED_BOUNDS_PRIORI_DECIDER 
→ SPEED_HEURISTIC_OPTIMIZER → SPEED_DECIDER → SPEED_BOUNDS_FINAL_DECIDER 
→ PIECEWISE_JERK_SPEED

LANE_FOLLOW_PATH 的 nudge 逻辑检测到了障碍物，但障碍物分布导致左右 nudge 矛盾，从而卡住不动。

跨周期的数据调用
```
周期 N:
  LANE_FOLLOW_PATH → 调用 PathBoundsDeciderUtil → 检测 nudge 方向
                   → 设置 blocking_obstacle_id (如果有)
  PATH_DECIDER     → 读取 blocking_obstacle_id 
                   → 设置 front_static_obstacle_id 和 counter
周期 N+1:
  LANE_BORROW_PATH → 读取上周期的 front_static_obstacle_id
                   → 如果 counter >= threshold，则尝试借道
```
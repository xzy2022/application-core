## Overview
本次修改解决了 LANE_FOLLOW_PATH 模块中的 nudge 冲突问题，使得当存在无法通过 nudge 解决的障碍物时，系统能够正确触发借道逻辑。

## Problem Statement
在施工场景中，障碍物分布可能导致：

某些障碍物需要 LEFT_NUDGE（向左微移绕过）
同时其他障碍物需要 RIGHT_NUDGE（向右微移绕过）
这种冲突导致车辆卡住，无法触发 LANE_BORROW_PATH 借道模块。

## Solution
在 LaneFollowPath::DecidePathBounds 函数中添加 nudge 冲突检测逻辑：

在 GetBoundaryFromStaticObstacles 调用后检查所有障碍物的 nudge 方向
如果同时存在 LEFT_NUDGE 和 RIGHT_NUDGE 且路径宽度不足，设置 blocking_obstacle_id
这将触发后续的 LANE_BORROW_PATH 模块进行借道决策
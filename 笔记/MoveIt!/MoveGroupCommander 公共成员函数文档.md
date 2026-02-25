# MoveGroupCommander 公共成员函数文档

## 公共成员函数列表

🔷`__init__()`

**函数原型**

```python
__init__(
    name: str,
    robot_description: str = "robot_description",
    ns: str = "",
    wait_for_servers: float = 5.0
)
```

**作用**

创建一个 **MoveGroupCommander 实例对象**，用于控制某个 planning group。

**参数解释**

- `name: str` - 规划组名称，需与 SRDF 中定义一致
- `robot_description: str = "robot_description"` - URDF 在 ROS 参数服务器中的参数名
- `ns: str = ""` - 命名空间（namespace）
- `wait_for_servers: float = 5.0` - 等待 move\_group action server 启动的时间（秒）

**返回值**

无显式返回值

**使用示例**

```python
import rospy
import moveit_commander

rospy.init_node("test_node")

moveit_commander.roscpp_initialize([])
group = moveit_commander.MoveGroupCommander("manipulator")
```

🔷`allow_looking()`

**函数原型**

```python
allow_looking(value: bool)
```

**作用**

启用/禁用运动规划时的环境观察功能。

**参数解释**

- `value: bool` - True 启用观察，False 禁用观察

**返回值**

无

**使用示例**

```python
group.allow_looking(True)
```

---

🔷`allow_replanning()`

**函数原型**

```python
allow_replanning(value: bool)
```

**作用**

启用/禁用运动规划失败时的重新规划功能。

**参数解释**

- `value: bool` - True 启用重新规划，False 禁用重新规划

**返回值**

无

**使用示例**

```python
group.allow_replanning(True)
```

---

🔷`attach_object()`

**函数原型**

```python
attach_object(
    object_name: str,
    link_name: str = "",
    touch_links: list = []
)
```

**作用**

将规划场景中的物体附加到机器人的某个连杆上。

**参数解释**

- `object_name: str` - 要附加的物体名称
- `link_name: str = ""` - 要附加到的连杆名称，默认为末端执行器连杆
- `touch_links: list = []` - 允许接触该物体的连杆列表

**返回值**

`bool` - 如果成功发送附加请求返回 True

**使用示例**

```python
group.attach_object("box", "gripper_link", ["finger_1", "finger_2"])
```

---

🔷`clear_max_cartesian_link_speed()`

**函数原型**

```python
clear_max_cartesian_link_speed()
```

**作用**

清除笛卡尔空间连杆最大速度限制。

**参数解释**

无

**返回值**

无

**使用示例**

```python
group.clear_max_cartesian_link_speed()
```

---

🔷`clear_path_constraints()`

**函数原型**

```python
clear_path_constraints()
```

**作用**

清除运动规划时的路径约束。

**参数解释**

无

**返回值**

无

**使用示例**

```python
group.clear_path_constraints()
```

---

🔷`clear_pose_target()`

**函数原型**

```python
clear_pose_target(end_effector_link: str)
```

**作用**

清除特定末端执行器的位姿目标。

**参数解释**

- `end_effector_link: str` - 末端执行器连杆名称

**返回值**

无

**使用示例**

```python
group.clear_pose_target("gripper_link")
```

---

🔷`clear_pose_targets()`

**函数原型**

```python
clear_pose_targets()
```

**作用**

清除所有末端执行器的位姿目标。

**参数解释**

无

**返回值**

无

**使用示例**

```python
group.clear_pose_targets()
```

---

🔷`clear_trajectory_constraints()`

**函数原型**

```python
clear_trajectory_constraints()
```

**作用**

清除运动规划时的轨迹约束。

**参数解释**

无

**返回值**

无

**使用示例**

```python
group.clear_trajectory_constraints()
```

---

🔷`compute_cartesian_path()`

**函数原型**

```python
compute_cartesian_path(
    waypoints: list,
    eef_step: float,
    avoid_collisions: bool = True,
    path_constraints = None
)
```

**作用**

计算末端执行器沿直线段移动的路径点序列。

**参数解释**

- `waypoints: list` - 位姿路径点列表
- `eef_step: float` - 每个路径点之间的步长（米）
- `avoid_collisions: bool = True` - 是否避免碰撞
- `path_constraints` - 路径约束（可选）

**返回值**

`tuple` - (RobotTrajectory 轨迹消息, 路径完成比例 float)

**使用示例**

```python
waypoints = [pose1, pose2, pose3]
(plan, fraction) = group.compute_cartesian_path(waypoints, 0.01)
```

---

🔷`construct_motion_plan_request()`

**函数原型**

```python
construct_motion_plan_request()
```

**作用**

返回一个填充了当前目标的 MotionPlanRequest 消息。

**参数解释**

无

**返回值**

`MotionPlanRequest` - 运动规划请求消息

**使用示例**

```python
request = group.construct_motion_plan_request()
```

---

🔷`detach_object()`

**函数原型**

```python
detach_object(name: str = "")
```

**作用**

从连杆上分离物体。

**参数解释**

- `name: str = ""` - 连杆名称或物体名称，为空则尝试分离所有物体

**返回值**

无

**使用示例**

```python
group.detach_object("box")
```

---

🔷`enforce_bounds()`

**函数原型**

```python
enforce_bounds(robot_state_msg)
```

**作用**

对 RobotState 消息强制执行状态边界约束。

**参数解释**

- `robot_state_msg` - moveit_msgs.RobotState 消息

**返回值**

无

**使用示例**

```python
from moveit_msgs.msg import RobotState
state = RobotState()
group.enforce_bounds(state)
```

---

🔷`execute()`

**函数原型**

```python
execute(trajectory, wait: bool = True)
```

**作用**

执行给定的轨迹。

**参数解释**

- `trajectory` - RobotTrajectory 轨迹消息
- `wait: bool = True` - 是否等待执行完成

**返回值**

`bool` - 执行成功返回 True

**使用示例**

```python
plan = group.plan()
group.execute(plan[1])
```

---

🔷`forget_joint_values()`

**函数原型**

```python
forget_joint_values(name: str)
```

**作用**

删除已存储的关节配置。

**参数解释**

- `name: str` - 存储的配置名称

**返回值**

无

**使用示例**

```python
group.forget_joint_values("home_position")
```

---

🔷`get_active_joints()`

**函数原型**

```python
get_active_joints()
```

**作用**

获取该组的活动关节列表。

**参数解释**

无

**返回值**

`list` - 活动关节名称列表

**使用示例**

```python
joints = group.get_active_joints()
```

---

🔷`get_current_joint_values()`

**函数原型**

```python
get_current_joint_values()
```

**作用**

获取该组当前的关节配置（从 /joint_states 发布的值）。

**参数解释**

无

**返回值**

`list` - 当前关节值列表

**使用示例**

```python
current_joints = group.get_current_joint_values()
```

---

🔷`get_current_pose()`

**函数原型**

```python
get_current_pose(end_effector_link: str = "")
```

**作用**

获取末端执行器的当前位姿。

**参数解释**

- `end_effector_link: str = ""` - 末端执行器连杆名称，默认使用组的末端执行器

**返回值**

`PoseStamped` - 当前位姿

**使用示例**

```python
current_pose = group.get_current_pose()
```

---

🔷`get_current_rpy()`

**函数原型**

```python
get_current_rpy(end_effector_link: str = "")
```

**作用**

获取末端执行器的当前 RPY（横滚、俯仰、偏航）角度。

**参数解释**

- `end_effector_link: str = ""` - 末端执行器连杆名称

**返回值**

`list` - [roll, pitch, yaw] 三个元素的列表

**使用示例**

```python
rpy = group.get_current_rpy()
```

---

🔷`get_current_state()`

**函数原型**

```python
get_current_state()
```

**作用**

获取机器人的当前状态。

**参数解释**

无

**返回值**

`RobotState` - 机器人状态消息

**使用示例**

```python
current_state = group.get_current_state()
```

---

🔷`get_current_state_bounded()`

**函数原型**

```python
get_current_state_bounded()
```

**作用**

获取机器人的当前状态（已应用边界约束）。

**参数解释**

无

**返回值**

`RobotState` - 机器人状态消息

**使用示例**

```python
bounded_state = group.get_current_state_bounded()
```

---

🔷`get_end_effector_link()`

**函数原型**

```python
get_end_effector_link()
```

**作用**

获取被视为末端执行器的连杆名称。

**参数解释**

无

**返回值**

`str` - 末端执行器连杆名称，如果没有则返回空字符串

**使用示例**

```python
eef_link = group.get_end_effector_link()
```

---

🔷`get_goal_joint_tolerance()`

**函数原型**

```python
get_goal_joint_tolerance()
```

**作用**

获取关节目标的容差（每个关节变量的距离）。

**参数解释**

无

**返回值**

`float` - 关节容差值

**使用示例**

```python
tolerance = group.get_goal_joint_tolerance()
```

---

🔷`get_goal_orientation_tolerance()`

**函数原型**

```python
get_goal_orientation_tolerance()
```

**作用**

获取目标姿态的方向容差（roll, pitch, yaw 到目标的距离）。

**参数解释**

无

**返回值**

`float` - 方向容差值

**使用示例**

```python
orientation_tol = group.get_goal_orientation_tolerance()
```

---

🔷`get_goal_position_tolerance()`

**函数原型**

```python
get_goal_position_tolerance()
```

**作用**

获取目标位置的容差（末端执行器目标原点周围的球体半径）。

**参数解释**

无

**返回值**

`float` - 位置容差值

**使用示例**

```python
position_tol = group.get_goal_position_tolerance()
```

---

🔷`get_goal_tolerance()`

**函数原型**

```python
get_goal_tolerance()
```

**作用**

返回目标容差的元组：关节、位置和方向。

**参数解释**

无

**返回值**

`tuple` - (关节容差, 位置容差, 方向容差)

**使用示例**

```python
tolerances = group.get_goal_tolerance()
```

---

🔷`get_interface_description()`

**函数原型**

```python
get_interface_description()
```

**作用**

获取规划器接口的描述（规划器 ID 列表）。

**参数解释**

无

**返回值**

`str` - 规划器接口描述

**使用示例**

```python
description = group.get_interface_description()
```

---

🔷`get_jacobian_matrix()`

**函数原型**

```python
get_jacobian_matrix(
    joint_values: list,
    reference_point = None
)
```

**作用**

获取该组的雅可比矩阵。

**参数解释**

- `joint_values: list` - 关节值列表
- `reference_point` - 参考点（可选）

**返回值**

`list` - 雅可比矩阵

**使用示例**

```python
jacobian = group.get_jacobian_matrix([0, 0, 0, 0, 0, 0])
```

---

🔷`get_joint_value_target()`

**函数原型**

```python
get_joint_value_target()
```

**作用**

获取当前设置的关节值目标。

**参数解释**

无

**返回值**

`list` - 关节目标值列表

**使用示例**

```python
target = group.get_joint_value_target()
```

---

🔷`get_joints()`

**函数原型**

```python
get_joints()
```

**作用**

获取该组的所有关节列表。

**参数解释**

无

**返回值**

`list` - 关节名称列表

**使用示例**

```python
joints = group.get_joints()
```

---

🔷`get_known_constraints()`

**函数原型**

```python
get_known_constraints()
```

**作用**

获取该组的已知约束名称列表（从数据库读取）。

**参数解释**

无

**返回值**

`list` - 约束名称列表

**使用示例**

```python
constraints = group.get_known_constraints()
```

---

🔷`get_name()`

**函数原型**

```python
get_name()
```

**作用**

获取该实例初始化时使用的组名称。

**参数解释**

无

**返回值**

`str` - 组名称

**使用示例**

```python
name = group.get_name()
```

---

🔷`get_named_target_values()`

**函数原型**

```python
get_named_target_values(target: str)
```

**作用**

获取命名目标的关节值字典。

**参数解释**

- `target: str` - 目标名称

**返回值**

`dict` - 关节名称到值的字典

**使用示例**

```python
values = group.get_named_target_values("home")
```

---

🔷`get_named_targets()`

**函数原型**

```python
get_named_targets()
```

**作用**

获取所有关节配置的名称列表。

**参数解释**

无

**返回值**

`list` - 配置名称列表

**使用示例**

```python
targets = group.get_named_targets()
```

---

🔷`get_path_constraints()`

**函数原型**

```python
get_path_constraints()
```

**作用**

获取实际的路径约束（moveit_msgs.msgs.Constraints 形式）。

**参数解释**

无

**返回值**

`Constraints` - 路径约束消息

**使用示例**

```python
constraints = group.get_path_constraints()
```

---

🔷`get_planner_id()`

**函数原型**

```python
get_planner_id()
```

**作用**

获取当前选择的规划管道的规划器 ID（如 RRTConnect, LIN）。

**参数解释**

无

**返回值**

`str` - 规划器 ID

**使用示例**

```python
planner = group.get_planner_id()
```

---

🔷`get_planning_frame()`

**函数原型**

```python
get_planning_frame()
```

**作用**

获取执行所有规划的坐标系名称。

**参数解释**

无

**返回值**

`str` - 规划坐标系名称

**使用示例**

```python
frame = group.get_planning_frame()
```

---

🔷`get_planning_pipeline_id()`

**函数原型**

```python
get_planning_pipeline_id()
```

**作用**

获取当前的规划管道 ID（如 ompl, pilz_industrial_motion_planner）。

**参数解释**

无

**返回值**

`str` - 规划管道 ID

**使用示例**

```python
pipeline = group.get_planning_pipeline_id()
```

---

🔷`get_planning_time()`

**函数原型**

```python
get_planning_time()
```

**作用**

获取用于运动规划的时间量。

**参数解释**

无

**返回值**

`float` - 规划时间（秒）

**使用示例**

```python
time = group.get_planning_time()
```

---

🔷`get_pose_reference_frame()`

**函数原型**

```python
get_pose_reference_frame()
```

**作用**

获取末端执行器位姿假定的参考坐标系。

**参数解释**

无

**返回值**

`str` - 参考坐标系名称

**使用示例**

```python
frame = group.get_pose_reference_frame()
```

---

🔷`get_random_joint_values()`

**函数原型**

```python
get_random_joint_values()
```

**作用**

获取随机的关节值配置。

**参数解释**

无

**返回值**

`list` - 随机关节值列表

**使用示例**

```python
random_joints = group.get_random_joint_values()
```

---

🔷`get_random_pose()`

**函数原型**

```python
get_random_pose(end_effector_link: str = "")
```

**作用**

获取末端执行器的随机位姿。

**参数解释**

- `end_effector_link: str = ""` - 末端执行器连杆名称

**返回值**

`PoseStamped` - 随机位姿

**使用示例**

```python
random_pose = group.get_random_pose()
```

---

🔷`get_remembered_joint_values()`

**函数原型**

```python
get_remembered_joint_values()
```

**作用**

获取该组的名称到关节配置的字典。

**参数解释**

无

**返回值**

`dict` - 名称到关节值的字典

**使用示例**

```python
remembered = group.get_remembered_joint_values()
```

---

🔷`get_trajectory_constraints()`

**函数原型**

```python
get_trajectory_constraints()
```

**作用**

获取实际的轨迹约束（moveit_msgs.msgs.TrajectoryConstraints 形式）。

**参数解释**

无

**返回值**

`TrajectoryConstraints` - 轨迹约束消息

**使用示例**

```python
constraints = group.get_trajectory_constraints()
```

---

🔷`get_variable_count()`

**函数原型**

```python
get_variable_count()
```

**作用**

返回用于参数化该组状态的变量数量（大于或等于自由度数）。

**参数解释**

无

**返回值**

`int` - 变量数量

**使用示例**

```python
count = group.get_variable_count()
```

---

🔷`go()`

**函数原型**

```python
go(joints = None, wait: bool = True)
```

**作用**

设置组的目标，然后移动组到指定目标。

**参数解释**

- `joints` - 关节值列表或字典（可选）
- `wait: bool = True` - 是否等待运动完成

**返回值**

`bool` - 运动成功返回 True

**使用示例**

```python
group.go()
```

---

🔷`has_end_effector_link()`

**函数原型**

```python
has_end_effector_link()
```

**作用**

检查该组是否有被视为末端执行器的连杆。

**参数解释**

无

**返回值**

`bool` - 有末端执行器返回 True

**使用示例**

```python
has_eef = group.has_end_effector_link()
```

---

🔷`limit_max_cartesian_link_speed()`

**函数原型**

```python
limit_max_cartesian_link_speed(
    speed: float,
    link_name: str = ""
)
```

**作用**

设置笛卡尔空间连杆的最大速度。

**参数解释**

- `speed: float` - 最大速度（米/秒），仅允许正实数
- `link_name: str = ""` - 连杆名称，默认为末端执行器

**返回值**

无

**使用示例**

```python
group.limit_max_cartesian_link_speed(0.1)
```

---

🔷`pick()`

**函数原型**

```python
pick(
    object_name: str,
    grasp = [],
    plan_only: bool = False
)
```

**作用**

拾取命名的物体。

**参数解释**

- `object_name: str` - 物体名称
- `grasp` - Grasp 消息或 Grasp 消息列表（可选）
- `plan_only: bool = False` - 仅规划不执行

**返回值**

`bool` - 成功返回 True

**使用示例**

```python
group.pick("box")
```

---

🔷`place()`

**函数原型**

```python
place(
    object_name: str,
    location = None,
    plan_only: bool = False
)
```

**作用**

将命名物体放置在环境中的特定位置，或在世界中的安全位置（如果未提供位置）。

**参数解释**

- `object_name: str` - 物体名称
- `location` - 放置位置（可选）
- `plan_only: bool = False` - 仅规划不执行

**返回值**

`bool` - 成功返回 True

**使用示例**

```python
group.place("box", location)
```

---

🔷`plan()`

**函数原型**

```python
plan(joints = None)
```

**作用**

返回运动规划结果的元组。

**参数解释**

- `joints` - 关节值列表或字典（可选）

**返回值**

`tuple` - (成功标志 bool, 轨迹消息 RobotTrajectory, 规划时间 float, 错误代码 MoveitErrorCodes)

**使用示例**

```python
success, plan, time, error = group.plan()
```

---

🔷`remember_joint_values()`

**函数原型**

```python
remember_joint_values(
    name: str,
    values = None
)
```

**作用**

以指定名称记录组的关节配置。

**参数解释**

- `name: str` - 配置名称
- `values` - 关节值（可选），未指定则记录当前状态

**返回值**

无

**使用示例**

```python
group.remember_joint_values("home")
```

---

🔷`retime_trajectory()`

**函数原型**

```python
retime_trajectory(
    ref_state_in,
    traj_in,
    velocity_scaling_factor: float = 1.0,
    acceleration_scaling_factor: float = 1.0,
    algorithm: str = "iterative_time_parameterization"
)
```

**作用**

对轨迹进行时间重参数化。

**参数解释**

- `ref_state_in` - 参考状态
- `traj_in` - 输入轨迹
- `velocity_scaling_factor: float = 1.0` - 速度缩放因子
- `acceleration_scaling_factor: float = 1.0` - 加速度缩放因子
- `algorithm: str` - 时间参数化算法

**返回值**

重新参数化后的轨迹

**使用示例**

```python
new_traj = group.retime_trajectory(ref_state, traj, 0.5, 0.5)
```

---

🔷`set_constraints_database()`

**函数原型**

```python
set_constraints_database(host: str, port: int)
```

**作用**

指定用于加载可能的路径约束的数据库连接。

**参数解释**

- `host: str` - 数据库主机地址
- `port: int` - 数据库端口

**返回值**

无

**使用示例**

```python
group.set_constraints_database("localhost", 33829)
```

---

🔷`set_end_effector_link()`

**函数原型**

```python
set_end_effector_link(link_name: str)
```

**作用**

设置被视为末端执行器的连杆名称。

**参数解释**

- `link_name: str` - 连杆名称

**返回值**

无

**使用示例**

```python
group.set_end_effector_link("gripper_link")
```

---

🔷`set_goal_joint_tolerance()`

**函数原型**

```python
set_goal_joint_tolerance(value: float)
```

**作用**

设置目标关节配置的容差。

**参数解释**

- `value: float` - 容差值

**返回值**

无

**使用示例**

```python
group.set_goal_joint_tolerance(0.01)
```

---

🔷`set_goal_orientation_tolerance()`

**函数原型**

```python
set_goal_orientation_tolerance(value: float)
```

**作用**

设置目标末端执行器方向的容差。

**参数解释**

- `value: float` - 容差值

**返回值**

无

**使用示例**

```python
group.set_goal_orientation_tolerance(0.01)
```

---

🔷`set_goal_position_tolerance()`

**函数原型**

```python
set_goal_position_tolerance(value: float)
```

**作用**

设置目标末端执行器位置的容差。

**参数解释**

- `value: float` - 容差值

**返回值**

无

**使用示例**

```python
group.set_goal_position_tolerance(0.01)
```

---

🔷`set_goal_tolerance()`

**函数原型**

```python
set_goal_tolerance(value: float)
```

**作用**

同时设置关节、位置和方向目标容差。

**参数解释**

- `value: float` - 容差值

**返回值**

无

**使用示例**

```python
group.set_goal_tolerance(0.01)
```

---

🔷`set_joint_value_target()`

**函数原型**

```python
set_joint_value_target(
    arg1,
    arg2 = None,
    arg3 = None
)
```

**作用**

为组指定目标关节配置。

**参数解释**

- `arg1` - 可以是 dict、list、JointState 消息、字符串（关节名）或 Pose/PoseStamped
- `arg2` - 当 arg1 为字符串时，为关节值；当 arg1 为 Pose 时，为末端执行器名称或布尔值
- `arg3` - 当 arg1 为 Pose 时，为布尔值（是否为近似位姿）

**返回值**

`bool` - 设置成功返回 True

**使用示例**

```python
# 使用列表
group.set_joint_value_target([0, 0, 0, 0, 0, 0])

# 使用字典
group.set_joint_value_target({"joint1": 0.5, "joint2": 1.0})

# 使用位姿（会调用 IK）
group.set_joint_value_target(pose)
```

---

🔷`set_max_acceleration_scaling_factor()`

**函数原型**

```python
set_max_acceleration_scaling_factor(value: float)
```

**作用**

设置缩放因子以降低最大关节加速度。

**参数解释**

- `value: float` - 缩放因子，允许值在 (0, 1] 范围内

**返回值**

无

**使用示例**

```python
group.set_max_acceleration_scaling_factor(0.5)
```

---

🔷`set_max_velocity_scaling_factor()`

**函数原型**

```python
set_max_velocity_scaling_factor(value: float)
```

**作用**

设置缩放因子以降低最大关节速度。

**参数解释**

- `value: float` - 缩放因子，允许值在 (0, 1] 范围内

**返回值**

无

**使用示例**

```python
group.set_max_velocity_scaling_factor(0.5)
```

---

🔷`set_named_target()`

**函数原型**

```python
set_named_target(name: str)
```

**作用**

按名称设置关节配置。

**参数解释**

- `name: str` - 配置名称（可以是之前用 remember_joint_values() 记录的名称或 SRDF 中指定的配置）

**返回值**

无

**使用示例**

```python
group.set_named_target("home")
```

---

🔷`set_num_planning_attempts()`

**函数原型**

```python
set_num_planning_attempts(num_planning_attempts: int)
```

**作用**

设置从头计算运动规划的次数，然后返回最短的解决方案。

**参数解释**

- `num_planning_attempts: int` - 规划尝试次数，默认值为 1

**返回值**

无

**使用示例**

```python
group.set_num_planning_attempts(10)
```

---

🔷`set_orientation_target()`

**函数原型**

```python
set_orientation_target(
    q,
    end_effector_link: str = ""
)
```

**作用**

为末端执行器指定目标方向（任何位置都可接受）。

**参数解释**

- `q` - 四元数 [x, y, z, w]
- `end_effector_link: str = ""` - 末端执行器连杆名称

**返回值**

无

**使用示例**

```python
group.set_orientation_target([0, 0, 0, 1])
```

---

🔷`set_path_constraints()`

**函数原型**

```python
set_path_constraints(value)
```

**作用**

指定要使用的路径约束（从数据库读取）。

**参数解释**

- `value` - 约束名称或 Constraints 消息

**返回值**

无

**使用示例**

```python
group.set_path_constraints("my_constraint")
```

---

🔷`set_planner_id()`

**函数原型**

```python
set_planner_id(planner_id: str)
```

**作用**

指定运动规划时使用当前选择管道的哪个规划器。

**参数解释**

- `planner_id: str` - 规划器 ID（如 RRTConnect, LIN）

**返回值**

无

**使用示例**

```python
group.set_planner_id("RRTConnect")
```

---

🔷`set_planning_pipeline_id()`

**函数原型**

```python
set_planning_pipeline_id(planning_pipeline: str)
```

**作用**

指定运动规划时使用哪个规划管道。

**参数解释**

- `planning_pipeline: str` - 规划管道 ID（如 ompl, pilz_industrial_motion_planner）

**返回值**

无

**使用示例**

```python
group.set_planning_pipeline_id("ompl")
```

---

🔷`set_planning_time()`

**函数原型**

```python
set_planning_time(seconds: float)
```

**作用**

指定用于运动规划的时间量。

**参数解释**

- `seconds: float` - 规划时间（秒）

**返回值**

无

**使用示例**

```python
group.set_planning_time(5.0)
```

---

🔷`set_pose_reference_frame()`

**函数原型**

```python
set_pose_reference_frame(reference_frame: str)
```

**作用**

设置末端执行器位姿假定的参考坐标系。

**参数解释**

- `reference_frame: str` - 参考坐标系名称

**返回值**

无

**使用示例**

```python
group.set_pose_reference_frame("base_link")
```

---

🔷`set_pose_target()`

**函数原型**

```python
set_pose_target(
    pose,
    end_effector_link: str = ""
)
```

**作用**

设置末端执行器的位姿目标。

**参数解释**

- `pose` - Pose 消息、PoseStamped 消息、6 个浮点数列表 [x, y, z, rot_x, rot_y, rot_z] 或 7 个浮点数列表 [x, y, z, qx, qy, qz, qw]
- `end_effector_link: str = ""` - 末端执行器连杆名称

**返回值**

`bool` - 设置成功返回 True

**使用示例**

```python
# 使用 Pose 消息
group.set_pose_target(pose_msg)

# 使用列表（位置 + 欧拉角）
group.set_pose_target([0.5, 0.5, 0.5, 0, 0, 0])

# 使用列表（位置 + 四元数）
group.set_pose_target([0.5, 0.5, 0.5, 0, 0, 0, 1])
```

---

🔷`set_pose_targets()`

**函数原型**

```python
set_pose_targets(
    poses: list,
    end_effector_link: str = ""
)
```

**作用**

设置末端执行器的多个位姿目标。

**参数解释**

- `poses: list` - 位姿列表，每个位姿可以是 Pose 消息、6 个浮点数列表或 7 个浮点数列表
- `end_effector_link: str = ""` - 末端执行器连杆名称

**返回值**

`bool` - 设置成功返回 True

**使用示例**

```python
poses = [pose1, pose2, pose3]
group.set_pose_targets(poses)
```

---

🔷`set_position_target()`

**函数原型**

```python
set_position_target(
    xyz: list,
    end_effector_link: str = ""
)
```

**作用**

为末端执行器指定目标位置（任何方向都可接受）。

**参数解释**

- `xyz: list` - 位置坐标 [x, y, z]
- `end_effector_link: str = ""` - 末端执行器连杆名称

**返回值**

无

**使用示例**

```python
group.set_position_target([0.5, 0.5, 0.5])
```

---

🔷`set_random_target()`

**函数原型**

```python
set_random_target()
```

**作用**

设置随机关节配置目标。

**参数解释**

无

**返回值**

无

**使用示例**

```python
group.set_random_target()
```

---

🔷`set_rpy_target()`

**函数原型**

```python
set_rpy_target(
    rpy: list,
    end_effector_link: str = ""
)
```

**作用**

为末端执行器指定目标方向（任何位置都可接受）。

**参数解释**

- `rpy: list` - [roll, pitch, yaw] 欧拉角列表
- `end_effector_link: str = ""` - 末端执行器连杆名称

**返回值**

无

**使用示例**

```python
group.set_rpy_target([0, 0, 1.57])
```

---

🔷`set_start_state()`

**函数原型**

```python
set_start_state(msg)
```

**作用**

为组指定起始状态。

**参数解释**

- `msg` - moveit_msgs/RobotState 消息

**返回值**

无

**使用示例**

```python
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

joint_state = JointState()
joint_state.header.stamp = rospy.Time.now()
joint_state.name = ['joint_a', 'joint_b']
joint_state.position = [0.17, 0.34]

moveit_robot_state = RobotState()
moveit_robot_state.joint_state = joint_state
group.set_start_state(moveit_robot_state)
```

---

🔷`set_start_state_to_current_state()`

**函数原型**

```python
set_start_state_to_current_state()
```

**作用**

将起始状态设置为当前状态。

**参数解释**

无

**返回值**

无

**使用示例**

```python
group.set_start_state_to_current_state()
```

---

🔷`set_support_surface_name()`

**函数原型**

```python
set_support_surface_name(value: str)
```

**作用**

为放置操作设置支撑表面名称。

**参数解释**

- `value: str` - 支撑表面名称

**返回值**

无

**使用示例**

```python
group.set_support_surface_name("table")
```

---

🔷`set_trajectory_constraints()`

**函数原型**

```python
set_trajectory_constraints(value)
```

**作用**

指定要使用的轨迹约束（从数据库设置尚未实现）。

**参数解释**

- `value` - TrajectoryConstraints 消息

**返回值**

无

**使用示例**

```python
group.set_trajectory_constraints(constraints)
```

---

🔷`set_workspace()`

**函数原型**

```python
set_workspace(ws: list)
```

**作用**

设置机器人的工作空间。

**参数解释**

- `ws: list` - 工作空间边界，可以是 []、[minX, minY, maxX, maxY] 或 [minX, minY, minZ, maxX, maxY, maxZ]

**返回值**

无

**使用示例**

```python
# 2D 工作空间
group.set_workspace([-1, -1, 1, 1])

# 3D 工作空间
group.set_workspace([-1, -1, 0, 1, 1, 2])
```

---

🔷`shift_pose_target()`

**函数原型**

```python
shift_pose_target(
    axis: int,
    value: float,
    end_effector_link: str = ""
)
```

**作用**

获取末端执行器的当前位姿，在相应轴上添加值，并将新位姿设置为位姿目标。

**参数解释**

- `axis: int` - 轴索引（0-5: X, Y, Z, R, P, Y）
- `value: float` - 要添加的值
- `end_effector_link: str = ""` - 末端执行器连杆名称

**返回值**

`bool` - 设置成功返回 True

**使用示例**

```python
# 在 X 轴上移动 0.1 米
group.shift_pose_target(0, 0.1)
```

---

🔷`stop()`

**函数原型**

```python
stop()
```

**作用**

停止当前执行（如果有）。

**参数解释**

无

**返回值**

无

**使用示例**

```python
group.stop()
```

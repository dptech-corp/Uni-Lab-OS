## 简单单变量动作函数


### `SendCmd`

```{literalinclude} ../../unilabos_msgs/action/SendCmd.action
:language: yaml
```

----
## 常量有机化学操作

Uni-Lab 常量有机化学指令集多数来自 [XDL](https://croningroup.gitlab.io/chemputer/xdl/standard/full_steps_specification.html#)，包含有机合成实验中常见的操作，如加热、搅拌、冷却等。



### `Clean`

```{literalinclude} ../../unilabos_msgs/action/Clean.action
:language: yaml
```

----

### `HeatChillStart`

```{literalinclude} ../../unilabos_msgs/action/HeatChillStart.action
:language: yaml
```

----

### `HeatChillStop`

```{literalinclude} ../../unilabos_msgs/action/HeatChillStop.action
:language: yaml
```

----

### `PumpTransfer`

```{literalinclude} ../../unilabos_msgs/action/PumpTransfer.action
:language: yaml
```

----
## 移液工作站及相关生物自动化设备操作

Uni-Lab 生物操作指令集多数来自 [PyLabRobot](https://docs.pylabrobot.org/user_guide/index.html)，包含生物实验中常见的操作，如移液、混匀、离心等。



### `LiquidHandlerDiscardTips`

```{literalinclude} ../../unilabos_msgs/action/LiquidHandlerDiscardTips.action
:language: yaml
```

----

### `LiquidHandlerDropTips`

```{literalinclude} ../../unilabos_msgs/action/LiquidHandlerDropTips.action
:language: yaml
```

----

### `LiquidHandlerDropTips96`

```{literalinclude} ../../unilabos_msgs/action/LiquidHandlerDropTips96.action
:language: yaml
```

----

### `LiquidHandlerMoveLid`

```{literalinclude} ../../unilabos_msgs/action/LiquidHandlerMoveLid.action
:language: yaml
```

----

### `LiquidHandlerMovePlate`

```{literalinclude} ../../unilabos_msgs/action/LiquidHandlerMovePlate.action
:language: yaml
```

----

### `LiquidHandlerMoveResource`

```{literalinclude} ../../unilabos_msgs/action/LiquidHandlerMoveResource.action
:language: yaml
```

----

### `LiquidHandlerPickUpTips`

```{literalinclude} ../../unilabos_msgs/action/LiquidHandlerPickUpTips.action
:language: yaml
```

----

### `LiquidHandlerPickUpTips96`

```{literalinclude} ../../unilabos_msgs/action/LiquidHandlerPickUpTips96.action
:language: yaml
```

----

### `LiquidHandlerReturnTips`

```{literalinclude} ../../unilabos_msgs/action/LiquidHandlerReturnTips.action
:language: yaml
```

----

### `LiquidHandlerReturnTips96`

```{literalinclude} ../../unilabos_msgs/action/LiquidHandlerReturnTips96.action
:language: yaml
```

----

### `LiquidHandlerStamp`

```{literalinclude} ../../unilabos_msgs/action/LiquidHandlerStamp.action
:language: yaml
```

----
## 多工作站及小车运行、物料转移


### `AGVTransfer`

```{literalinclude} ../../unilabos_msgs/action/AGVTransfer.action
:language: yaml
```

----

### `WorkStationRun`

```{literalinclude} ../../unilabos_msgs/action/WorkStationRun.action
:language: yaml
```

----
## 机械臂、夹爪等机器人设备

Uni-Lab 机械臂、机器人、夹爪和导航指令集沿用 ROS2 的 `control_msgs` 和 `nav2_msgs`：


### `FollowJointTrajectory`

```yaml
# The trajectory for all revolute, continuous or prismatic joints
trajectory_msgs/JointTrajectory trajectory
# The trajectory for all planar or floating joints (i.e. individual joints with more than one DOF)
trajectory_msgs/MultiDOFJointTrajectory multi_dof_trajectory

# Tolerances for the trajectory.  If the measured joint values fall
# outside the tolerances the trajectory goal is aborted.  Any
# tolerances that are not specified (by being omitted or set to 0) are
# set to the defaults for the action server (often taken from the
# parameter server).

# Tolerances applied to the joints as the trajectory is executed.  If
# violated, the goal aborts with error_code set to
# PATH_TOLERANCE_VIOLATED.
JointTolerance[] path_tolerance
JointComponentTolerance[] component_path_tolerance

# To report success, the joints must be within goal_tolerance of the
# final trajectory value.  The goal must be achieved by time the
# trajectory ends plus goal_time_tolerance.  (goal_time_tolerance
# allows some leeway in time, so that the trajectory goal can still
# succeed even if the joints reach the goal some time after the
# precise end time of the trajectory).
#
# If the joints are not within goal_tolerance after "trajectory finish
# time" + goal_time_tolerance, the goal aborts with error_code set to
# GOAL_TOLERANCE_VIOLATED
JointTolerance[] goal_tolerance
JointComponentTolerance[] component_goal_tolerance
builtin_interfaces/Duration goal_time_tolerance

---
int32 error_code
int32 SUCCESSFUL = 0
int32 INVALID_GOAL = -1
int32 INVALID_JOINTS = -2
int32 OLD_HEADER_TIMESTAMP = -3
int32 PATH_TOLERANCE_VIOLATED = -4
int32 GOAL_TOLERANCE_VIOLATED = -5

# Human readable description of the error code. Contains complementary
# information that is especially useful when execution fails, for instance:
# - INVALID_GOAL: The reason for the invalid goal (e.g., the requested
#   trajectory is in the past).
# - INVALID_JOINTS: The mismatch between the expected controller joints
#   and those provided in the goal.
# - PATH_TOLERANCE_VIOLATED and GOAL_TOLERANCE_VIOLATED: Which joint
#   violated which tolerance, and by how much.
string error_string

---
std_msgs/Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint desired
trajectory_msgs/JointTrajectoryPoint actual
trajectory_msgs/JointTrajectoryPoint error

string[] multi_dof_joint_names
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_desired
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_actual
trajectory_msgs/MultiDOFJointTrajectoryPoint multi_dof_error

```

----
### `GripperCommand`

```yaml
GripperCommand command
---
float64 position  # The current gripper gap size (in meters)
float64 effort    # The current effort exerted (in Newtons)
bool stalled      # True iff the gripper is exerting max effort and not moving
bool reached_goal # True iff the gripper position has reached the commanded setpoint
---
float64 position  # The current gripper gap size (in meters)
float64 effort    # The current effort exerted (in Newtons)
bool stalled      # True iff the gripper is exerting max effort and not moving
bool reached_goal # True iff the gripper position has reached the commanded setpoint

```

----
### `JointTrajectory`

```yaml
trajectory_msgs/JointTrajectory trajectory
---
---

```

----
### `ParallelGripperCommand`

```yaml
# Parallel grippers refer to an end effector where two opposing fingers grasp an object from opposite sides.
sensor_msgs/JointState command
# name: the name(s) of the joint this command is requesting
# position: desired position of each gripper joint (radians or meters)
# velocity: (optional, not used if empty) max velocity of the joint allowed while moving (radians or meters / second)
# effort: (optional, not used if empty) max effort of the joint allowed while moving (Newtons or Newton-meters)
---
sensor_msgs/JointState state # The current gripper state.
# position of each joint (radians or meters)
# optional: velocity of each joint (radians or meters / second)
# optional: effort of each joint (Newtons or Newton-meters)
bool stalled      # True if the gripper is exerting max effort and not moving
bool reached_goal # True if the gripper position has reached the commanded setpoint
---
sensor_msgs/JointState state # The current gripper state.
# position of each joint (radians or meters)
# optional: velocity of each joint (radians or meters / second)
# optional: effort of each joint (Newtons or Newton-meters)

```

----
### `PointHead`

```yaml
geometry_msgs/PointStamped target
geometry_msgs/Vector3 pointing_axis
string pointing_frame
builtin_interfaces/Duration min_duration
float64 max_velocity
---
---
float64 pointing_angle_error

```

----
### `SingleJointPosition`

```yaml
float64 position
builtin_interfaces/Duration min_duration
float64 max_velocity
---
---
std_msgs/Header header
float64 position
float64 velocity
float64 error

```

----
### `AssistedTeleop`

```yaml
#goal definition
builtin_interfaces/Duration time_allowance
---
#result definition
builtin_interfaces/Duration total_elapsed_time
---
#feedback
builtin_interfaces/Duration current_teleop_duration

```

----
### `BackUp`

```yaml
#goal definition
geometry_msgs/Point target
float32 speed
builtin_interfaces/Duration time_allowance
---
#result definition
builtin_interfaces/Duration total_elapsed_time
---
#feedback definition
float32 distance_traveled

```

----
### `ComputePathThroughPoses`

```yaml
#goal definition
geometry_msgs/PoseStamped[] goals
geometry_msgs/PoseStamped start
string planner_id
bool use_start # If false, use current robot pose as path start, if true, use start above instead
---
#result definition
nav_msgs/Path path
builtin_interfaces/Duration planning_time
---
#feedback definition

```

----
### `ComputePathToPose`

```yaml
#goal definition
geometry_msgs/PoseStamped goal
geometry_msgs/PoseStamped start
string planner_id
bool use_start # If false, use current robot pose as path start, if true, use start above instead
---
#result definition
nav_msgs/Path path
builtin_interfaces/Duration planning_time
---
#feedback definition

```

----
### `DriveOnHeading`

```yaml
#goal definition
geometry_msgs/Point target
float32 speed
builtin_interfaces/Duration time_allowance
---
#result definition
builtin_interfaces/Duration total_elapsed_time
---
#feedback definition
float32 distance_traveled

```

----
### `DummyBehavior`

```yaml
#goal definition
std_msgs/String command
---
#result definition
builtin_interfaces/Duration total_elapsed_time
---
#feedback definition

```

----
### `FollowPath`

```yaml
#goal definition
nav_msgs/Path path
string controller_id
string goal_checker_id
---
#result definition
std_msgs/Empty result
---
#feedback definition
float32 distance_to_goal
float32 speed

```

----
### `FollowWaypoints`

```yaml
#goal definition
geometry_msgs/PoseStamped[] poses
---
#result definition
int32[] missed_waypoints
---
#feedback definition
uint32 current_waypoint

```

----
### `NavigateThroughPoses`

```yaml
#goal definition
geometry_msgs/PoseStamped[] poses
string behavior_tree
---
#result definition
std_msgs/Empty result
---
#feedback definition
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
float32 distance_remaining
int16 number_of_poses_remaining

```

----
### `NavigateToPose`

```yaml
#goal definition
geometry_msgs/PoseStamped pose
string behavior_tree
---
#result definition
std_msgs/Empty result
---
#feedback definition
geometry_msgs/PoseStamped current_pose
builtin_interfaces/Duration navigation_time
builtin_interfaces/Duration estimated_time_remaining
int16 number_of_recoveries
float32 distance_remaining

```

----
### `SmoothPath`

```yaml
#goal definition
nav_msgs/Path path
string smoother_id
builtin_interfaces/Duration max_smoothing_duration
bool check_for_collisions
---
#result definition
nav_msgs/Path path
builtin_interfaces/Duration smoothing_duration
bool was_completed
---
#feedback definition

```

----
### `Spin`

```yaml
#goal definition
float32 target_yaw
builtin_interfaces/Duration time_allowance
---
#result definition
builtin_interfaces/Duration total_elapsed_time
---
#feedback definition
float32 angular_distance_traveled

```

----
### `Wait`

```yaml
#goal definition
builtin_interfaces/Duration time
---
#result definition
builtin_interfaces/Duration total_elapsed_time
---
#feedback definition
builtin_interfaces/Duration time_left

```

----

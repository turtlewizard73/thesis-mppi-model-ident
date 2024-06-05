# Controller benchmark
`ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False`

## MPPI
- motion model: differentail drive

## MPPI Critics tests
**Need for common metrics:**
*goal*: by looking at a few diagrams / numbers you can tell difference between good or bad parameters/settings/tuning
- success
  - quantized: true/false
  - vicinity to goal - closer=better
  - elapsed time - shorter=better
    - max time: slowest speed * plan = t
    - min time: fastest speed
- trajectory
  - similarity to the original (nem teljesen másfele ment)
    - only when there are no obstacles
  - https://www.tandfonline.com/doi/full/10.1080/15481603.2021.1908927
  - angle of attack vector not changing rapidly (nem kacsázik)
- driving
  - average velocity (linear, angular) - how close is it to max
  - avarage acceleration
  - avarage rms jerk (=mennyire hirtelen változik a gyorsulás) - lower=beter

### Constraint Critic
This critic penalizes trajectories that have components outside of the set dynamic or kinematic constraints.
**Test:** cmd_vel -> twist -> velocities

- cost_weight: Weight to apply to critic term.
- cost_power: Power order to apply to term.

### Goal Angle Critic
This critic incentivizes navigating to achieve the angle of the goal pose when in reasonable proximity to goal.
**Test:** odom -> last pose -> compare it to goal

- cost_weight: Weight to apply to critic term.
- cost_power: Power order to apply to term.
- threshold_to_consider: Minimal distance (m) between robot and goal above which angle goal cost considered.

### Goal Critic
This critic incentivizes navigating spatially towards the goal when in reasonable proximity to goal.
**Test:** ???

- cost_weight: Weight to apply to critic term.
- cost_power: Power order to apply to term.
- threshold_to_consider: Minimal distance (m) between robot and goal above which goal distance cost considered. It is wise to start with this as being the same as your prediction horizon to have a clean hand-off with the path follower critic.

### Obstacles Critic
Not used -> Cost Critic

### Cost Critic
This critic incentivizes navigating away from obstacles and critical collisions using either a circular robot point-check or full SE2 footprint check using the costmap values.
**Test:** costmap -> get cost for each pose -> avarage cost along route

- cost_weight: Weight to apply to critic.
- cost_power: Power order to apply to term.
- consider_footprint: Whether to use point cost (if robot is circular or low compute power) or compute SE2 footprint cost.
- collision_cost: Cost to apply to a true collision in a trajectory.
- critical_cost: Cost to apply to a pose with any point in in inflated space to prefer distance from obstacles.
- near_goal_distance: Distance (m) near goal to stop applying preferential obstacle term to allow robot to smoothly converge to goal pose in close proximity to obstacles.
- inflation_layer_name: Name of the inflation layer. If empty, it uses the last inflation layer in the costmap. If you have multiple inflation layers, you may want to specify the name of the layer to use.
- trajectory_point_step: The step to take in trajectories for evaluating them in the critic. Since trajectories are extremely dense, its unnecessary to evaluate each point and computationally expensive.

### Path Align Critic
This critic incentivizes aligning with the global path, if relevant. It does not implement path following behavior.
**Test:** odometry -> poses through route -> similarity???

- cost_weight: Weight to apply to critic term.
- cost_power: Power order to apply to term.
- threshold_to_consider: Distance (m) between robot and goal to stop considering path alignment and allow goal critics to take over.
- offset_from_furthest: Checks that the candidate trajectories are sufficiently far along their way tracking the path to apply the alignment critic. This ensures that path alignment is only considered when actually tracking the path, preventing awkward initialization motions preventing the robot from leaving the path to achieve the appropriate heading.
- max_path_occupancy_ratio: Maximum proportion of the path that can be occupied before this critic is not considered to allow the obstacle and path follow critics to avoid obstacles while following the path’s intent in presence of dynamic objects in the scene. Between 0-1 for 0-100%.
- use_path_orientations: Whether to consider path’s orientations in path alignment, which can be useful when paired with feasible smac planners to incentivize directional changes only where/when the smac planner requests them. If you want the robot to deviate and invert directions where the controller sees fit, keep as false. If your plans do not contain orientation information (e.g. navfn), keep as false.
- trajectory_point_step: The step to take in trajectories for evaluating them in the critic. Since trajectories are extremely dense, its unnecessary to evaluate each point and computationally expensive.

### Path Angle Critic
This critic penalizes trajectories at a high relative angle to the path. This helps the robot make sharp turns when necessary due to large accumulated angular errors.
**Test:** check cmd_vel attack angle at every pose?

- cost_weight: Weight to apply to critic term.
- cost_power: Power order to apply to term.
- threshold_to_consider: Distance (m) between robot and goal to stop considering path angles and allow goal critics to take over.
- offset_from_furthest: Number of path points after furthest one any trajectory achieves to compute path angle relative to.
- max_angle_to_furthest: Angular distance (rad) between robot and goal above which path angle cost starts being considered.
- mode: Enum type for mode of operations for the path angle critic depending on path input types and behavioral desires.

### Path Follow Critic
This critic incentivizes making progress along the path. This is what drives the robot forward along the path.
**Test:** Robot moves -> elapsed time

- cost_weight: Weight to apply to critic term.
- cost_power: Power order to apply to term.
- threshold_to_consider: Distance (m) between robot and goal to stop considering path following and allow goal critics to take over. It is wise to start with this as being the same as your prediction horizon to have a clean hand-off with the goal critic.
- offset_from_furthest: Number of path points after furthest one any trajectory achieves to drive path tracking relative to.

### Prefer Forward Critic
This critic incentivizes moving in the forward direction, rather than reversing.
**Test:** cmd_vel -> twist -> compare +/- velocites

- cost_weight: Weight to apply to critic term.
- cost_power: Power order to apply to term.
- threshold_to_consider: Distance (m) between robot and goal to stop considering preferring forward and allow goal critics to take over.

### Twirling Critic
This critic penalizes unnecessary ‘twisting’ with holonomic vehicles. It adds a constraint on the rotation angle to keep it consistent.
**Test:** cmd_vel -> twist -> compare angular velocities
or should be ignored since we are not holonomic

- cost_weight: Weight to apply to critic term.
- cost_power: Power order to apply to term.

### Velocity Deadband Critic
This critic penalizes velocities that fall below the deadband threshold, helping to mitigate hardware limitations on certain platforms.
**Test:** cmd_vel -> twist -> velocities >= deadband

- cost_weight: Weight to apply to critic term.
- cost_power: Power order to apply to term.
- deadband_velocities: The array of deadband velocities [vx, vz, wz]. A zero array indicates that the critic will take no action.


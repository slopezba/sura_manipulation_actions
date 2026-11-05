# Empty arrays use configured defaults. Base and tip arrays have six entries;
# joint arrays follow the complete selected arm's joint_names order.
# Base errors are world XYZ/RPY; tip errors are XYZ/rotation-vector components
# in the frozen target reference axes. Explicit zero is a strict tolerance.
float64[] base_path
float64[] base_goal
float64[] joint_path
float64[] joint_goal
float64[] tip_path
float64[] tip_goal
# Zero uses the configured convergence allowance.
builtin_interfaces/Duration goal_time_tolerance

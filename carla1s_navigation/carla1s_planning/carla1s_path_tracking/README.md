# Path Tracking

## ROS Interface

### Topic Subscribers and Publishers

|Topic Name|Type|
|---|---|
|/carla/{role_name}/vehicle_info|carla_msgs::CarlaEgoVehicleInfo|
|/carla/{role_name}/odometry|nav_msgs/Odometry|
|/carla/{role_name}/vehicle_control_cmd|carla_msgs/CarlaEgoVehicleControl|
|/carla1s/{role_name}/path_tracking/debug_markers|visualization_msgs/MarkerArray|
|/carla1s/{role_name}/path_tracking/current_path|nav_msgs/Path|
|/carla1s/{role_name}/path_tracking/debug_data|std_msgs/Float64|

### Action Server

|Action|Type|
|---|---|
|/carla1s/{role_name}/path_tracking_action|carla1s_msgs/PathTrackingAction|
# ROS parameters

All distances are expressed in metres, angles in radians, durations in seconds,
and frequencies in hertz unless stated otherwise. Ranges document the intended
operating domain; runtime validation will be added separately.

## `fm2_controller_node.py`

| Parameter | Unit / range | Default | Description |
|---|---:|---:|---|
| `~frame_map` | non-empty ROS frame ID | `map` | Global frame used by paths, goals, and poses. |
| `~lookahead` | m, > 0 | `0.35` | Path look-ahead distance. |
| `~v_lin` | m/s, >= 0 | `0.22` | Nominal linear velocity. |
| `~v_ang_max` | rad/s, > 0 | `1.5` | Maximum tracking angular velocity. |
| `~heading_align_threshold` | rad, [0, pi] | `0.35` | Heading error above which translation stops. |
| `~align_v_ang_max` | rad/s, > 0 | `0.8` | Maximum angular velocity during heading alignment. |
| `~goal_tolerance` | m, >= 0 | `0.08` | Position tolerance at the goal. |
| `~rate` | Hz, > 0 | `20` | Control-loop frequency. |
| `~k_theta` | 1/s, >= 0 | `2.0` | Proportional heading gain. |
| `~goal_yaw_tolerance` | rad, >= 0 | `0.10` | Final orientation tolerance. |
| `~use_goal_yaw` | boolean | `true` | Enable final goal-orientation alignment. |

## `fm2_costmap_node.py`

| Parameter | Unit / range | Default | Description |
|---|---:|---:|---|
| `~frame_map` | non-empty ROS frame ID | `map` | Costmap output frame. |
| `~map_topic` | valid ROS topic name | `/map` | Static occupancy-grid input. |
| `~scan_topic` | valid ROS topic name | `/scan` | Laser scan input. |
| `~person_tracks_topic` | valid ROS topic name | `/person_tracks` | Tracked-person input. |
| `~obstacle_range` | m, > 0 | `2.5` | Maximum laser obstacle range. |
| `~min_range` | m, >= 0 | `0.05` | Minimum accepted laser range. |
| `~dynamic_inflate` | cells, integer >= 0 | `0` | Dynamic-obstacle inflation radius. |
| `~person_radius` | m, >= 0 | `0.35` | Physical safety radius around a person. |
| `~person_inflate` | cells, integer >= 0 | `2` | Additional person-layer inflation. |
| `~person_prediction_enabled` | boolean | `true` | Enable local constant-velocity projection. |
| `~person_predictions_topic` | valid ROS topic name | `/person_predictions` | External trajectory-prediction input. |
| `~person_predictions_enabled` | boolean | `false` | Enable the external prediction layer. |
| `~prediction_sigma_multiplier` | dimensionless, >= 0 | `1.0` | Scale applied to prediction uncertainty. |
| `~prediction_max_longitudinal_radius` | m, >= 0 | `0.90` | Maximum longitudinal prediction radius. |
| `~prediction_max_lateral_radius` | m, >= 0 | `0.55` | Maximum lateral prediction radius. |
| `~person_predictions_timeout` | s, >= 0 | `0.6` | External-prediction freshness timeout. |
| `~person_use_confirmed_only` | boolean | `true` | Ignore unconfirmed person tracks. |
| `~person_prediction_horizons` | list of s, each > 0 | `[0.5, 1.0, 1.5, 2.0]` | Local projection horizons. |
| `~person_tracks_timeout` | s, >= 0 | `0.6` | Track-layer freshness timeout. |
| `~person_max_speed_warn` | m/s, > 0 | `1.5` | Speed threshold for warning logs. |
| `~dynamic_memory` | scans, integer [0, 255] | `15` | Lifetime counter for laser obstacles. |

## `fm2_planner_node.py`

| Parameter | Unit / range | Default | Description |
|---|---:|---:|---|
| `~frame_map` | non-empty ROS frame ID | `map` | Global planning frame. |
| `~frame_base` | non-empty ROS frame ID | `base_link` | Robot base frame used as a pose fallback. |
| `~inflate` | cells, integer >= 0 | `0` | Planning-map inflation radius. |
| `~replan_offpath` | m, >= 0 | `0.6` | Distance that triggers off-path replanning. |
| `~replan_period` | s, >= 0 | `1.0` | Periodic replanning interval; zero disables it. |
| `~rate` | Hz, > 0 | `20` | Planner-loop frequency. |

## `simple_nav_node.py`

| Parameter | Unit / range | Default | Description |
|---|---:|---:|---|
| `~frame_map` | non-empty ROS frame ID | `map` | Global frame. |
| `~replan_period` | s, >= 0 | `0.2` | Periodic replanning interval. |
| `~inflate` | cells, integer >= 0 | `2` | Planning-map inflation radius. |
| `~lookahead` | m, > 0 | `0.35` | Path look-ahead distance. |
| `~v_lin` | m/s, >= 0 | `0.22` | Nominal linear velocity. |
| `~v_ang_max` | rad/s, > 0 | `1.5` | Maximum angular velocity. |
| `~goal_tolerance` | m, >= 0 | `0.08` | Position tolerance at the goal. |
| `~k_theta` | 1/s, >= 0 | `2.0` | Proportional heading gain. |
| `~replan_offpath` | m, >= 0 | `1.0` | Distance that triggers off-path replanning. |
| `~rate` | Hz, > 0 | `20` | Control-loop frequency. |

## `initial_pose_publisher.py`

| Parameter | Unit / range | Default | Description |
|---|---:|---:|---|
| `~frame_id` | non-empty ROS frame ID | `map` | Frame of the initial pose. |
| `~x` | m, finite | `0.0` | Initial X coordinate. |
| `~y` | m, finite | `0.0` | Initial Y coordinate. |
| `~yaw` | rad, finite | `0.0` | Initial heading. |
| `~repeats` | count, integer >= 1 | `5` | Number of publications. |
| `~rate` | Hz, > 0 | `1.0` | Publication frequency. |
| `~std_xy` | m, >= 0 | `0.10` | Position standard deviation. |
| `~std_yaw` | rad, >= 0 | `0.10` | Heading standard deviation. |

## `person_patrol_mover.py`

| Parameter | Unit / range | Default | Description |
|---|---:|---:|---|
| `~model_name` | non-empty Gazebo model name | `person_target` | Model moved along the patrol. |
| `~speed` | m/s, >= 0 | `0.3` | Patrol speed. |
| `~z_fixed` | m, finite | `0.0` | Fixed model height. |
| `~rate` | Hz, > 0 | `20.0` | Update frequency. |
| `~waypoint_tolerance` | m, >= 0 | `0.05` | Waypoint acceptance radius. |
| `~waypoints` | list of at least two finite `[x, y]` pairs, m | `[[1, 1], [1, -1], [-1, -1], [-1, 1]]` | Closed patrol route. |

## `person_track_publisher.py`

| Parameter | Unit / range | Default | Description |
|---|---:|---:|---|
| `~person_model_name` | non-empty Gazebo model name | `person_target` | Source model. |
| `~output_frame` | non-empty ROS frame ID | `map` | Frame declared on output tracks. |
| `~track_id` | integer >= 0 | `1` | Published track identifier. |
| `~velocity_smoothing` | dimensionless, [0, 1] | `0.5` | Exponential velocity-smoothing factor. |
| `~publish_rate` | Hz, > 0 | `15.0` | Maximum publication frequency. |
| `~max_speed_warn` | m/s, > 0 | `1.5` | Speed threshold for warning logs. |
| `~min_dt` | s, > 0 | `0.01` | Minimum interval used for velocity estimation. |

## `zed_body_tracking_sim_node.py`

| Parameter | Unit / range | Default | Description |
|---|---:|---:|---|
| `~person_tracks_topic` | valid ROS topic name | `/person_tracks` | Simulated tracking output. |
| `~output_frame` | non-empty ROS frame ID | `map` | Frame declared on output tracks. |
| `~camera_frame` | non-empty ROS frame ID | `zed_sim_camera_frame` | Camera frame used for visibility checks. |
| `~person_model_prefixes` | non-empty list of non-empty strings | `[person_target, person_]` | Gazebo model-name filters. |
| `~sample_rate` | Hz, > 0 | `15.0` | Sensor sampling frequency. |
| `~min_range` | m, >= 0 | `0.5` | Minimum detection range. |
| `~max_range` | m, > min range | `8.0` | Maximum detection range. |
| `~horizontal_fov_deg` | degrees, (0, 360] | `110.0` | Horizontal field of view. |
| `~simulate_sensor_noise` | boolean | `false` | Enable stochastic sensor effects. |
| `~detection_probability` | probability, [0, 1] | `0.96` | Base detection probability. |
| `~distance_probability_drop` | probability, [0, 1] | `0.35` | Detection-probability loss at maximum range. |
| `~occlusion_probability` | probability, [0, 1] | `0.05` | Random occlusion probability. |
| `~position_noise_std` | m, >= 0 | `0.06` | Position-noise standard deviation. |
| `~velocity_noise_std` | m/s, >= 0 | `0.08` | Velocity-noise standard deviation. |
| `~latency` | s, >= 0 | `0.10` | Output latency. |
| `~tracking_timeout` | s, > 0 | `0.50` | Track retention time after a missed detection. |
| `~velocity_smoothing` | dimensionless, [0, 1] | `0.5` | Velocity-smoothing factor. |
| `~random_seed` | any integer | `7` | Seed for deterministic random sampling. |

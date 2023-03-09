-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  --map_builder = MAP_BUILDER,
  --trajectory_builder = TRAJECTORY_BUILDER,
  --map_frame = "map",
  --tracking_frame = "imu_link",
  --published_frame = "odom",
  --odom_frame = "odom",
  --provide_odom_frame = true,
  --publish_frame_projected_to_2d = false,
  --use_odometry = false,
  --use_nav_sat = false,
  --use_landmarks = false,
  --num_laser_scans = 1,
  --num_multi_echo_laser_scans = 0,
  --num_subdivisions_per_laser_scan = 1,
  --num_point_clouds = 0,
  --lookup_transform_timeout_sec = 0.2,
  --submap_publish_period_sec = 0.3,
  --pose_publish_period_sec = 5e-3,
  --trajectory_publish_period_sec = 30e-3,
  --rangefinder_sampling_ratio = 1.,
  --odometry_sampling_ratio = 0.3,
  --fixed_frame_pose_sampling_ratio = 1.,
  --imu_sampling_ratio = 0.1,
  --landmarks_sampling_ratio = 1.,

  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom_frame",
  publish_frame_projected_to_2d = false,
  provide_odom_frame = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.5,
  trajectory_publish_period_sec = 0.5,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.05,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.03,
  landmarks_sampling_ratio = 1.,
}

--TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 270

-- TRAJECTORY_BUILDER_2D.use_imu_data = false
--TRAJECTORY_BUILDER_2D.max_range = 10
MAP_BUILDER.use_trajectory_builder_3d = false
MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 10.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025



-- TRAJECTORY_BUILDER_2D.submaps.resolution = 0.025

-- POSE_GRAPH.constraint_builder.min_score = 0.65
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7


--MAP_BUILDER.num_background_threads = 7
--POSE_GRAPH.optimization_problem.huber_scale = 5e2
--POSE_GRAPH.optimize_every_n_nodes = 320
--POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
--POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
--POSE_GRAPH.constraint_builder.min_score = 0.62
--POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

return options

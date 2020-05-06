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

include "rover.lua"

TRAJECTORY_BUILDER.pure_localization = true

POSE_GRAPH.optimize_every_n_nodes = 10 
MAP_BUILDER.num_background_threads = 6
POSE_GRAPH.global_sampling_ratio = 0.0005
POSE_GRAPH.constraint_builder.sampling_ratio = 0.0005
POSE_GRAPH.constraint_builder.min_score = 0.72
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.76
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 5
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.075
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 10

return options

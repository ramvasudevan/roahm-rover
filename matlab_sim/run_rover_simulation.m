%% description
% This script runs a simulation with the ROAHM rover in the simulator
% framework, using RTD to plan online.
%
% 14 Mar 2020
%
%% user parameters
% world
obstacle_size_bounds = [0.3, 0.3] ; % side length [min, max]
N_obstacles = 6 ;
bounds = [-8,4,-2,2] ;
goal_radius = 0.5 ;
world_buffer = 0.75 ; % this is used to make start and goal locations 
                      % that are not too close to the robot or boundary

% planner
obs_buffer = 0.1 ; % m (Represents d*obs_point_spacing, with 'd' as in FRS_intersect_ipopt.py)
obs_point_spacing = 0.1; % m (Represents the grid resolution)
t_plan = 0.5 ; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ;

% simulation
verbose_level = 5 ;
max_sim_iterations = 100 ;
max_sim_time = 100 ;

%% automated from here
A = rover_agent ;

P = rover_RTD_planner_static('verbose',verbose_level,'buffer',obs_buffer,...
                             'point_spacing',obs_point_spacing,'t_plan',t_plan,'t_move',t_move,...
                             'plot_HLP_flag',true) ;
                                   
W = static_box_world('bounds',bounds,'N_obstacles',N_obstacles,'buffer',world_buffer,...
                     'verbose',verbose_level,'goal_radius',goal_radius,...
                     'obstacle_size_bounds',obstacle_size_bounds) ;

S = simulator(A,W,P,'allow_replan_errors',true,'verbose',verbose_level,...
              'max_sim_time',30,'max_sim_iterations',60) ;

%% run simulation
S.run ;

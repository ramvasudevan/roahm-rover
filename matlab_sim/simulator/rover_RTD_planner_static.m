classdef rover_RTD_planner_static < planner
% Class: rover_RTD_planner_static < planner
%
% This class implements RTD for a rover in static environments. It does
% not inherit the generic RTD planner superclass, so that you can see how
% all the parts of a planner should be written in a simulator framework.
%
% Adopted from turtlebot_RTD_planner_static
% 14 Mar 2020

    %% properties
    properties
    % inherited properties with default values (see planner.m)
        % name
        % bounds % world bounds plus planner-specific buffer
        % buffer % minimum amount to buffer obstacles, given by the world
        % HLP % high level planner
        % current_plan ;
        % current_obstacles ;
        % verbose = 0 ;
        % timeout = 1 ; % time allowed for "replan" function to execute
        % t_plan = 1 ; % same as timeout; just for notational purposes
        % t_move = 1 ;% amount of time the planner expects the agent to move
        % info % information structure to keep a log when planning
        % plot_data % data for current plot
        % plot_waypoints_flag = false ; 
    
        % FRS handling
        FRS % this is a cell array of loaded info from FRS .mat files
        FRS_degree = 10 ; % set to 4, 6, 8, 10, or 12
        FRS_polynomial_structure
        
        % obstacle handling
        point_spacing
        current_obstacles_raw
        current_obstacles_in_FRS_coords
        bounds_as_obstacle
        
        % plan handling
        current_waypoint
        lookahead_distance = 1.5 ;
        
        % plotting
        plot_obstacles_flag = true ;
        plot_waypoints_flag = true;
        plot_FRS_flag = true ;
    end
    
    %% methods
    methods
    %% constructor
        function P = rover_RTD_planner_static(varargin)
            % P = turtlebot_RTD_planner_static(varargin)
            %
            % This constructs the RTD planner.
            
            % set default values of some properties for the superclass that
            % are relevant to this particular planner; note that other
            % properties are defined in the planner "setup" method
            name = 'Rover RTD Planner' ;
            HLP = straight_line_HLP() ; % default high level planner
            
            % parse the input arguments; these should be given in the
            % format 'property1', value1, 'property2', value2,...
            P = parse_args(P,'name',name,'HLP',HLP,...
                           varargin{:}) ;
            
            % load FRS files
            FRS_data = cell(1,1);
            FRS_data{1} = load(...
                '/home/steven/roahm_lab/RTD_sim/rover/full_parameter_space/rover_FRS_prestruct_deg_10.mat');
            P.FRS = FRS_data ;
       
        end
        
    %% setup
        function setup(P,agent_info,world_info)
            % P.setup(agent_info, world_info)
            %
            % This is used to set stuff up before planning. For RTD, we use
            % it for the following:
            %   1. compute the obstacle discretization point spacing
            %   2. set up world boundaries as an obstacle
            %   3. give the high level planner the global goal info
            %   4. decompose the FRS polynomial into a usable form
            
            P.vdisp('Running setup',3)
            
        %% 1. compute point spacing
            P.vdisp('Ensuring buffer feasible',4)
            
            res = P.point_spacing;
            W = agent_info.footprint(2);

            a = res/2;
            if (W <= res)
               P.vdisp('BUFFER NOT FEASIBLE')
            end
            d = P.buffer*res;
            buf = res+(d-1)*res;
            if buf <= a
                P.vdisp('BUFFER NOT FEASIBLE')
            end
                        
        %% 2. set up world boundaries as an obstacle
            P.vdisp('Setting up world bounds as an obstacle',4)
        
            P.bounds = world_info.bounds + P.buffer.*[1 -1 1 -1] ;
            
            % create world bounds as an obstacle; note this passes the
            % bounds in as a clockwise polyline, so everything outside of
            % the world bounds counts as inside the polyline if using
            % functions like inpolygon
            xlo = P.bounds(1) ; xhi = P.bounds(2) ;
            ylo = P.bounds(3) ; yhi = P.bounds(4) ;
            
            B = [xlo, xhi, xhi, xlo, xlo ;
                ylo, ylo, yhi, yhi, ylo] ;
            B = [B, nan(2,1), 1.01.*B(:,end:-1:1)] ;
            
            P.bounds_as_obstacle = B ;
            
        %% 3. set up high level planner
            P.vdisp('Setting up high-level planner',4)
            
            P.HLP.default_lookahead_distance = P.lookahead_distance ;
            
        %% 4. initialize the current plan as empty
            P.vdisp('Initializing current plan',4)
            
            P.current_plan.T = [] ;
            P.current_plan.U = [] ;
            P.current_plan.Z = [] ;
            
        %% 5. clear plot data
            P.plot_data.obstacles = [] ;
            P.plot_data.waypoint = [] ;
            P.plot_data.waypoints = [] ;
            P.plot_data.FRS = [] ;
            
        %% 6. set up info structure to save replan dat
            I = struct('agent_time',[],'agent_state',[],'agent_future_state',[],...
                'k_opt_found',[-1;0],...
                'FRS_index',[],...
                'waypoint',[],...
                'waypoints',[],...
                'obstacles',[],...
                'obstacles_in_world_frame',[],...
                'obstacles_in_FRS_frame',[]) ;
            P.info = I ;
            P.current_plan.U = [0;0];
        end
        
    %% replan
        function [T,U,Z] = replan(P,agent_info,world_info)
            % [T,U,Z] = P.replan(agent_info,world_info)
            %
            % This is the core of the RTD planner. In this method, we
            % generate a new trajectory plan, or continue the old plan, by
            % using the FRS to identify unsafe plans, and then using
            % an optimization program to find a safe plan.
            
            P.vdisp('Planning!',3)
            start_tic = tic;
            
        %% 1. Get poses and get FRS
            P.vdisp('Getting poses',4)
            
            current_FRS_index = 1;
            FRS_cur = P.FRS{current_FRS_index} ;
            U = P.current_plan.U;
            
            % (x,y,h,v) Current State            
            agent_state = agent_info.state(:,end) ; 
            
            % Get FRS_poly here
            FRS_msspoly = FRS_cur.FRS_polynomial-1;
            k = FRS_cur.k;
            z = FRS_cur.z;
            FRS_poly = get_FRS_polynomial_structure(FRS_msspoly,z,k) ;     
            
        %% 2. process obstacles
            P.vdisp('Processing obstacles',4)
        
            O = world_info.obstacles ;
            
            % add world bounds as obstacle
            O = [O, nan(2,1), P.bounds_as_obstacle] ;
            
            % buffer and discretize obstacles
            [O_FRS, ~, O_pts] = compute_turtlebot_discretized_obs(O,...
                    agent_state,P.buffer,P.point_spacing,FRS_cur) ;
            
            % save obstacles
            P.current_obstacles_raw = O ; % input from the world
            P.current_obstacles = O_pts ; % buffered and discretized
            P.current_obstacles_in_FRS_coords = O_FRS ;
        
        %% 3. create the cost function for fmincon
            P.vdisp('Creating cost function',4)
            
            % buffer obstacles for high-level planner
            O_HLP = buffer_polygon_obstacles(O,agent_info.footprint(2)) ;
            world_info.obstacles = O_HLP ;
            
            % make a waypoint (this is wrapped in a try/catch in case the
            % waypoint planner has bugs)
            try
                P.vdisp('Getting waypoint',7)
                z_goal = P.HLP.get_waypoint(agent_info,world_info,P.lookahead_distance) ;
            catch
                P.vdisp('Waypoint creation errored! Using global goal instead',6)
                z_goal = P.HLP.goal ;
            end
            P.current_waypoint = z_goal ;
            
            % put waypoint in robot's body-fixed frame to use for planning
            z_goal_local = world_to_FRS(z_goal,agent_state,...
                FRS_cur.initial_x,FRS_cur.initial_y,FRS_cur.distance_scale);
            
            % create cost function
            cost = @(k) rover_cost_for_fmincon(k,z_goal_local) ;
        
        %% 4. create the constraints for fmincon
            P.vdisp('Creating constraints',4)
        
            % create nonlinear constraints from the obstacles
            if ~isempty(O_FRS)
                % remove NaNs
                O_log = isnan(O_FRS(1,:)) ;
                O_FRS = O_FRS(:,~O_log) ;
                
                % plug in to FRS polynomial
                cons_poly = evaluate_FRS_polynomial_on_obstacle_points(FRS_poly,O_FRS) ;

                % get the gradient of the constraint polynomials
                cons_poly_grad = get_constraint_polynomial_gradient(cons_poly) ;
                
                % create constraint function
                nonlcon = @(k) turtlebot_nonlcon_for_fmincon(k,...
                                  cons_poly,cons_poly_grad,...
                                  start_tic,P.t_plan) ;
            else
                % if there are no obstacles then we don't need to consider
                % any constraints
                nonlcon = [] ;
            end
            
            % create bounds for k
            k_1_bounds = [-1,1] ;
            k_2_bounds = [-1,1];

            % combine bounds
            k_bounds = [k_1_bounds ; k_2_bounds] ;
            
        %% 5. call trajectory optimization
            P.vdisp('Running trajectory optimization',4)
            
            % create initial guess
            initial_guess =[-1;0] ;

            % create optimization options
            options =  optimoptions('fmincon',...
                            'MaxFunctionEvaluations',1e5,...
                            'MaxIterations',1e5,...
                            'OptimalityTolerance',1e-3',...
                            'CheckGradients',false,...
                            'FiniteDifferenceType','central',...
                            'Diagnostics','off',...
                            'SpecifyConstraintGradient',true,...
                            'SpecifyObjectiveGradient',false);
                        
            % call fmincon, with a try/catch since the cost and constraint
            % functions bail out by erroring if the timeout is reached
            try
                [k_opt,~,exitflag] = fmincon(cost,...
                                            initial_guess,...
                                            [],[],... % linear inequality constraints
                                            [],[],... % linear equality constraints
                                            k_bounds(:,1),... % lower bounds
                                            k_bounds(:,2),... % upper bounds
                                            nonlcon,...
                                            options) ;
            catch
                exitflag = -1 ;
                k_opt = P.info.k_opt_found;
            end
        
        %% 6. make the new plan 
            P.vdisp('Creating plan from trajopt result',4)
                        
            % if fmincon was successful, create a new plan
            if exitflag > 0
                P.vdisp('New plan successfully found!',5)
                U = [k_opt(1)*FRS_cur.k_scale(1) + FRS_cur.k_const;...
                    k_opt(2)*FRS_cur.k_scale(2)];

                % create the desired trajectory
                [T,Z] = make_rover_desired_trajectory(FRS_cur.t_plan,...
                            U,1,1,1,[0;0;0;agent_state(4)]) ;
                        
                Z = Z';        
                % move plan to world coordinates
                Z(1:3,:) = local_to_world(agent_state,Z(1:3,:)) ;
            else
                % brake
                k_opt = [-FRS_cur.k_const/FRS_cur.k_scale(1);k_opt(2)];
                U = [0;U(2)];
                [T,Z] = make_rover_desired_trajectory(FRS_cur.t_stop,...
                    U,1,1,1,[0;0;0;agent_state(4)]); 
                
                Z = Z';
                % move plan to world coordinates
                Z(1:3,:) = local_to_world(agent_state,Z(1:3,:)) ;
            end
            
            % For the case where LLC is open_loop
            U = [U(1)*ones(1,size(T,2));U(2)*ones(1,size(T,2))];
            
            % save the new plan
            P.current_plan.T = T ;
            P.current_plan.U = U ;
            P.current_plan.Z = Z ;
            
        %% 7. update the info structure
            I = P.info ;
            
            I.agent_time = [I.agent_time, agent_info.time(end)] ;
            I.agent_state = [I.agent_state, agent_state] ;
            I.k_opt_found = [I.k_opt_found, k_opt] ;
            I.FRS_index = [I.FRS_index, current_FRS_index] ;
            I.waypoint = [I.waypoint, z_goal] ;
            I.waypoints = [I.waypoints, {P.HLP.waypoints}] ;
            I.obstacles = [I.obstacles, {O}] ;
            I.obstacles_in_world_frame = [I.obstacles_in_world_frame, {O_pts}] ;
            I.obstacles_in_FRS_frame = [I.obstacles_in_FRS_frame, {O_FRS}] ;
            
            P.info = I ;
            
        end
        
        %% plotting
        function plot(P,~)
            P.vdisp('Plotting!',8)
            P.plot_at_time() ;
        end
        
        function plot_at_time(P,t)            
            if nargin < 2
                if ~isempty(P.info.agent_time)
                    t = P.info.agent_time(end) ;
                else
                    t = 0 ;
                end
            end
            
            hold_check = false ;
            if ~ishold
                hold_check = true ;
                hold on ;
            end
            
            % figure out the info index closest to the current time
            I = P.info ;
            info_idx = find(t >= I.agent_time,1,'last') ;
            info_idx_check = ~isempty(info_idx) ;
            
            % plot current obstacles
            if P.plot_obstacles_flag && info_idx_check
                O = I.obstacles_in_world_frame{info_idx} ;

                if isempty(O)
                    O = nan(2,1) ;       
                end

                if check_if_plot_is_available(P,'obstacles')
                    P.plot_data.obstacles.XData = O(1,:) ;
                    P.plot_data.obstacles.YData = O(2,:) ;
                else
                    obs_data = plot(O(1,:),O(2,:),'r.') ;
                    P.plot_data.obstacles = obs_data ;
                end
            end
            
            % plot current waypoint
            if P.plot_waypoints_flag && info_idx_check
                wp = I.waypoint(:,info_idx) ;
                if isempty(wp)
                    wp = nan(2,1) ;
                end

                if check_if_plot_is_available(P,'waypoint')
                    P.plot_data.waypoint.XData = wp(1) ;
                    P.plot_data.waypoint.YData = wp(2) ;
                else
                    wp_data = plot(wp(1),wp(2),'b*') ;
                    P.plot_data.waypoint = wp_data ;
                end
            end
            
            % plot FRS
            if P.plot_FRS_flag && info_idx_check
                % iterate back through the info indices until the last
                % info index where k_opt was found
                FRS_info_idx = info_idx ;
                k_opt_idx = nan(2,1);
                while FRS_info_idx > 0
                    k_opt_idx = I.k_opt_found(:,FRS_info_idx) ;
                    if ~isnan(k_opt_idx(1))
                        break
                    else
                        FRS_info_idx = FRS_info_idx - 1 ;
                    end
                end
                
                % get the FRS and agent state for the current info index
                if ~isempty(FRS_info_idx) && FRS_info_idx > 0 && ~isnan(k_opt_idx(1))
                    FRS_idx = P.FRS{I.FRS_index(FRS_info_idx)} ;
                    agent_state = I.agent_state(:,FRS_info_idx) ;
                    
                    if check_if_plot_is_available(P,'FRS')
                        % get polynomial sliced by k_opt
                        FRS_poly = msubs(FRS_idx.FRS_polynomial,FRS_idx.k,k_opt_idx) ;
                        
                        % get the 2D contour points to plot
                        [~,FRS_patch_info,N] = get_2D_contour_points(FRS_poly,FRS_idx.z,1,'Bounds',0.9) ;
                        
                        % get the contour with the most vertices
                        [~,plot_idx] = max(N) ;
                        FRS_patch_info = FRS_patch_info(plot_idx) ;
                        
                        % put the vertices in the world frame
                        V = FRS_patch_info.Vertices ;
                        V = FRS_to_world(V',agent_state,...
                            FRS_idx.initial_x,FRS_idx.initial_y,FRS_idx.distance_scale)' ;
                        
                        P.plot_data.FRS.Faces = FRS_patch_info.Faces ;
                        P.plot_data.FRS.Vertices = V ;
                    else
                        if ~isnan(k_opt_idx(1))
                            FRS_data = plot_turtlebot_FRS_in_world_frame(FRS_idx,...
                                k_opt_idx,agent_state,...
                                'FaceColor',[0.5 1 0.3],'FaceAlpha',0.2,...
                                'EdgeColor',[0 0.6 0],'EdgeAlpha',0.5') ;
                            P.plot_data.FRS = FRS_data ;
                        end
                    end
                end
            end
            
            % plot high-level planner
            if P.plot_HLP_flag && info_idx_check
                % plot(P.HLP)
                waypoints = I.waypoints{info_idx} ;
                
                if isempty(waypoints)
                    waypoints = nan(2,1) ;
                end
                
                if check_if_plot_is_available(P,'waypoints')
                    P.plot_data.waypoints.XData = waypoints(1,:) ;
                    P.plot_data.waypoints.YData = waypoints(2,:) ;
                else
                    wps_data = plot_path(waypoints,'--','Color',[0.7 0.5 0.2]) ;
                    P.plot_data.waypoints = wps_data;
                end
            end
            
            if hold_check
                hold off
            end
        end
    end
end
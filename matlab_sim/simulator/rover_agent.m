classdef rover_agent < RTD_agent_2D
% Class: rover_agent < RTD_agent_2D < agent
%
% This implements the rover robot 

% Note that we treat yaw rate as the first input and acceleration as the
% second input.

% Adopted from turtlebot_agent.m
% 14 Mar 2020
    
    properties
        % state limits
        max_speed = 2 ; % m/s (NOTE this is higher than in the specs, since
                        % the planner should not command speeds above its
                        % own limit, but the robot is not limited by the
                        % planning algorithm)
        
        % state indices
        speed_index = 4 ;
        
        % integrator type, to allow for fixed time step integration
        integrator_type = 'ode45' ; % choose 'ode45' or 'ode4' or 'ode113'
        integrator_time_discretization = 0.01 ; % for ode4
                          
    end
    
    methods
        %% constructor
        function A = rover_agent(varargin)
            % set up default superclass values
            rover_length = .55 ; 
            rover_width = .3;
            default_footprint = [rover_length, rover_width] ;
            n_states = 4 ;
            n_inputs = 2 ;
            stopping_time = 2.63 ;
            sensor_radius = 6 ;
            LLC = open_loop_LLC ;
            
            % create agent
            A@RTD_agent_2D('footprint',default_footprint,...
                'n_states',n_states,'n_inputs',n_inputs,...
                'stopping_time',stopping_time,'sensor_radius',sensor_radius,...
                'LLC',LLC,varargin{:}) ;
        end
        
        %% emergency stop
        % note, this ignores any previous trajectory the agent may have
        % been tracking; we have to define this different from the default
        % for the TurtleBot because it takes in acceleration as a control
        % input, as opposed to doing feedback about desired speed
%         function stop(A,t_stop)
%             if nargin < 2
%                 t_stop = A.stopping_time ;
%             end
%             
%             % get the current speed
%             v = A.state(A.speed_index,end) ;
%             
%             % get braking input
%             u_stop = -A.max_accel ;
%             
%             if v == 0
%                 % if we are already stopped, stay stopped
%                 T_input = [0, t_stop] ;
%                 U_input = zeros(2) ;
%             else
%                 % check how long it will take to come to a stop
%                 t_req_to_stop = v/A.max_accel ;
% 
%                 if t_req_to_stop < t_stop
%                     T_input = [0, t_req_to_stop, t_stop] ;
%                     U_input = [0 0 0 ;
%                                u_stop, 0, 0] ;
%                 else
%                     T_input = [0, t_stop] ;
%                     U_input = [0 0 ;
%                                u_stop, 0] ;
%                 end
%             end
%             
%             % call move method to perform braking
%             A.move(t_stop,T_input,U_input) ;
%         end
        
        %% dynamics
        function zd = dynamics(A,t,z,T,U,Z)
            % handle no desired trajectory input
            if nargin < 6
                Z = [] ;
            end
            
            % get nominal control inputs
            u = A.LLC.get_control_inputs(A,t,z,T,U,Z) ;
            zd = zeros(4,1);
            
            c = [1.6615e-05,-1.9555e-07,3.6190e-06,4.3820e-07,-0.0811,...
       -    1.4736,0.1257,0.0765,-0.0140];
      
            zd(3) = ((tan(c(1)*u(2)+c(2))*z(4))/(c(3)+c(4)*z(4)^2));
            v_y = zd(3)*(c(8)+c(9)*z(4)^2) ;
            zd(1) = z(4)*cos(z(3))-v_y*sin(z(3));
            zd(2) = z(4)*sin(z(3))+v_y*cos(z(3));
            zd(4) = c(5)+c(6)*(z(4)-u(1))+c(7)*(z(4)-u(1))^2 ;
            
        end
        %% integrator options
        function [tout,zout] = integrator(A,fun,tspan,z0)
            switch A.integrator_type
                case 'ode45'
                    [tout,zout] = ode45(@(t,z) fun(t,z),tspan,z0(:)) ;
                case 'ode113'
                    [tout,zout] = ode113(@(t,z) fun(t,z),tspan,z0(:)) ;
                case {'ode4','RK4'}
                    dt = A.integrator_time_discretization ;
                    tout = tspan(1):dt:tspan(end) ;
                    if tout(end) ~= tspan(end)
                        tout = [tout, tspan(end)] ;
                    end
                    zout = ode4(@(t,z) fun(t,z),tout,z0(:)) ;
                otherwise
                    error('Please set A.integrator_type to either ode45 or ode4')
            end
            tout = tout(:)' ;
            zout = zout' ;
        end
    end
end

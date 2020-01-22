function [T,Z] = make_rover_desired_trajectory(t_f,U_in,fid,scale,distance_scale,init_cond)

    % set up timing
    t_sample = 0.01 ;
    T = unique([0:t_sample:t_f,t_f]);
    
    % compute desired trajectory
    if fid == 0
        z0 = zeros(2,1) ;
        if nargin == 6
            z0 = init_cond;
            [~,Z] = ode45(@(t,z) rover_trajectory_producing_model(t,z,U_in,scale,distance_scale),...
                T,z0) ;
        else
            [~,Z] = ode45(@(t,z) rover_trajectory_producing_model(t,z,U_in,scale,distance_scale),...
                T,z0) ;
        end
    elseif fid == 1
        z0 = zeros(4,1) ;
        if nargin == 6
            z0 = init_cond;
            [~,Z] = ode45(@(t,z) rover_tracking_model(t,z,U_in,scale,distance_scale),...
                T,z0) ;
        else
            [~,Z] = ode45(@(t,z) rover_tracking_model(t,z,U_in,scale,distance_scale),...
                T,z0) ;
        end
    end
    
end

%% Estimate of braking characteristics

% user parameters
% trajectory information
max_speed = 2 ; % m/s
t_plan = 0.5 ;



% set initial state (x,y,h,v)
z0 = [0;0;0;max_speed] ;
U_in = [0,0]; % command 0 speed.

[T,Z] = ode45(@(t,z) rover_tracking_model(t,z,U_in,1,1),[0 30],z0) ;

% get braking time as the first time the agent's speed falls below 1e-3 m/s
% and round up to the nearest 0.01s
stop_log = find(Z(:,4) <= 1e-3) ;
stop_idx = stop_log(1) ;
t_stop = T(stop_idx) ;
t_stop = round(t_stop,2) ;

% get braking distance as in (85)
d_stop = norm([Z(stop_idx,1),Z(stop_idx,2)] - [0,0]) ;

% get time required to travel the braking distance at max speed as in (91)
t_v = d_stop / max_speed ;

% output time horizon as in (92), rounding up to nearest 0.01s
t_f = round(t_plan + t_v,2) ;
disp(['Minimum planning time horizon: ',num2str(t_f,'%0.2f'),' seconds'])

%% save info
disp('Saving timing information to rover_timing.mat')
save('rover_timing.mat','t_f','t_plan','t_stop')



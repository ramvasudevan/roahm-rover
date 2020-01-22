% Verify Scaling

%% user parameters
t_f = 1.1 ;
U_in = [2 0.34];

%%
load rover_timing.mat
time_scale = t_f;

% Rover footprint is a box
rover_length = .55 ; 
wb = .32; % wheelbase, measured
rover_width = .3; 

T_ = 1;
U = [2 0] ; % 2m/s speed, strait ahead. this will choose dx as distance scale.
z0 = zeros(2,1); 
[~,Z] = ode45(@(t,z) rover_trajectory_producing_model(t,z,U,1,1),[0 t_f],z0) ;
dx = max(Z(:,1)) - min(Z(:,1)) + rover_length ;
dy = max(Z(:,2)) - min(Z(:,2)) + rover_width ;
distance_scale = max(dx,dy) ;

scale = (time_scale/distance_scale);
%% 
[T,Z] = make_rover_desired_trajectory(t_f,U_in,0,1,1) ;
[Th,Zh] = make_rover_desired_trajectory(t_f,U_in,1,1,1,[0;0;0;U_in(1)]) ;
[T_scale,Z_scale] = make_rover_desired_trajectory(T_,U_in,0,scale,distance_scale) ;
[Th_scale,Zh_scale] = make_rover_desired_trajectory(T_,U_in,1,scale,distance_scale,[0;0;0;U_in(1)]) ;

% get the x and y positions of the trajectory
x = Z(:,1) ;
y = Z(:,2) ;
xh = Zh(:,1) ;
yh = Zh(:,2) ;
x_scale = Z_scale(:,1) ;
y_scale = Z_scale(:,2) ;
xh_scale = Zh_scale(:,1) ;
yh_scale = Zh_scale(:,2) ;

% plot
figure(1) ;
hold on
plot(x,y,'b--','LineWidth',1.5)
plot(xh,yh,'m--','LineWidth',1.5);
plot(x_scale,y_scale,'b','LineWidth',1.5);
plot(xh_scale,yh_scale,'m','LineWidth',1.5);
plot(x/distance_scale,y/distance_scale,'bo','LineWidth',1.5); % This is what we do online.
legend('low unscaled','high unscaled','low scaled','high scaled','low correct scale')
axis equal


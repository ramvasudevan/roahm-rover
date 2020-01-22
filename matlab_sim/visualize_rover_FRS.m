%% user parameters
% trajectory parameter to evaluate, try extremes to get a feel of FRS.
k_eval = [0.65;-0.67] ;

%% automated from here
% load timing
load('rover_timing.mat')

% load FRSes

FRS_ = load('mat_files/rover_FRS_prestruct_deg_10.mat') ;
        
% get one h_Z0 to plot
h_Z0 = FRS_.h_Z0 ;
z = FRS_.z ;
k = FRS_.k ;
k_scale = FRS_.k_scale ;
k_const = FRS_.k_const ;
distance_scale = FRS_.distance_scale;
init_x = FRS_.initial_x;
init_y = FRS_.initial_y;
wid = FRS_.rover_width ;
len = FRS_.rover_length ;

scale = t_f/distance_scale;
T_ = 1;
%% create agent for visualization

U0 = k(1)*k_scale(1) + k_const;
U1 = k(2)*k_scale(2);

% create w_des and v_des from k_eval
U_in = [full(msubs(U0,k,k_eval));full(msubs(U1,k,k_eval))] ;

% create the desired trajectory
[~,Z_max] = make_rover_desired_trajectory(t_plan,U_in,1,1,1,[0;0;0;U_in(1)]) ;
[~,Z_max_brake] = make_rover_desired_trajectory(t_stop,[0,U_in(2)],1,1,1,...
    [0;0;0;Z_max(end,4)]);
[~,Z_min] = make_rover_desired_trajectory(t_plan,U_in,1,1,1,[0;0;0;0]) ;
[~,Z_min_brake] = make_rover_desired_trajectory(t_stop,[0,U_in(2)],1,1,1,...
    [0;0;0;Z_min(end,4)]);

[~,Z_low] = make_rover_desired_trajectory(T_,U_in,0,scale,distance_scale);

% convert to map frame at position t_plan
H_max = [cos(Z_max(end,3)), -sin(Z_max(end,3));...
     sin(Z_max(end,3)), cos(Z_max(end,3))];
 
pos_rowformat_max = H_max*Z_max_brake(:,1:2)';

H_min = [cos(Z_min(end,3)), -sin(Z_min(end,3));...
     sin(Z_min(end,3)), cos(Z_min(end,3))];
 
pos_rowformat_min = H_min*Z_min_brake(:,1:2)';

%% plot in FRS frame
figure(1) ; clf ; hold on ; axis equal

% plot degree 8 FRS
I_z_8 = msubs(FRS_.FRS_polynomial,k,k_eval) ;
plot_2D_msspoly_contour_rover(I_z_8,z,1,'LineWidth',1.5,'Color',0.6*[0.1 1 0.3],...
    'FillColor',[0.9 0.9 0.9])

% plot the highlighted trajectories
% t_f trajectory
plot(init_x+Z_low(:,1),init_y+Z_low(:,2),'b','LineWidth',1.5);

% plot the agent
plot(init_x+([-len len len -len -len]/(distance_scale*2)),...
    init_y+([-wid -wid wid wid -wid]/(distance_scale*2)),'b')

% plot the box_FRS
plot([-0.45 -0.45 0.80 0.80 -0.45],...
    [-0.85 0.85 0.85 -0.85 -0.85],'r')

% labeling
xlabel('x [FRS scale]')
ylabel('y [FRS scale]')
legend('FRS contour','low fid, t-f duration','rover footprint','box FRS','Location','Northwest')
set(gca,'FontSize',15)

axis([-1, 1, -1, 1])

%% plot in World frame
figure(2) ; clf ; hold on ; axis equal

plot_2D_msspoly_contour_rover(I_z_8,z,1,'LineWidth',1.5,'Color',0.6*[0.1 1 0.3],...
    'FillColor',[0.9 0.9 0.9],'Scale',distance_scale)

% plot the highlighted trajectories
% t_plan trajectory
plot(init_x*distance_scale+Z_max(:,1),init_y+Z_max(:,2),'b','LineWidth',1.5);
% braking trajectory
plot(init_x*distance_scale+Z_max(end,1)+pos_rowformat_max(1,:)',...
    init_y+Z_max(end,2)+pos_rowformat_max(2,:)','b--','LineWidth',1.5);
% t_plan trajectory
plot(init_x*distance_scale+Z_min(:,1),init_y+Z_min(:,2),'k','LineWidth',1.5);
% braking trajectory
plot(init_x*distance_scale+Z_min(end,1)+pos_rowformat_min(1,:)',...
    init_y+Z_min(end,2)+pos_rowformat_min(2,:)','k--','LineWidth',1.5);

% plot the agent
plot(init_x*distance_scale+([-len len len -len -len]/2),...
    init_y*distance_scale+([-wid -wid wid wid -wid]/2),'b')

% labeling
xlabel('x [WORLD scale]')
ylabel('y [WORLD scale]')
legend('FRS contour','high fid, t-plan duration, init commanded speed','high fid, t-stop duration, braking',...
    'high fid, t-plan duration, init 0 speed','high fid, t-stop duration, braking',...
    'Location','Northwest')
set(gca,'FontSize',15)





















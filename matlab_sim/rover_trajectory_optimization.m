%% user parameters

% robot desired location %IN FRS FRAME
x_des = 0.75 ;
y_des = -0.2 ;

% create waypoint from desired location
z_goal_local = [x_des; y_des] ;

% obstacles, in column format
O_FRS = [1 0 0 0.5 0.55 0.45;0 1 -0.2 0.06 0.02 -0.1];

%% automated from here
% load FRS
load('full_parameter_space/rover_FRS_deg_10_matlab.mat') ; % You HAVE subtracted 1.

%% create cost function
% use waypoint to make cost function
cost = @(k) rover_cost_for_fmincon(k,z_goal_local) ;

%% create constraint function

% evaluate the FRS polynomial structure input on the obstacle points to get
% the list of constraint polynomials
cons_poly = evaluate_FRS_polynomial_on_obstacle_points(FRS_poly,O_FRS) ;

% get the gradient of the constraint polynomials
cons_poly_grad = get_constraint_polynomial_gradient(cons_poly) ;

% create nonlinear constraint function for fmincon
% ok to use turtlebot one
nonlcon = @(k) turtlebot_nonlcon_for_fmincon(k,cons_poly,cons_poly_grad) ;

% create bounds for k
k_1_bounds = [-1,1];
k_2_bounds = [-1,1];


% combine bounds
k_bounds = [k_1_bounds ; k_2_bounds] ;

%% run trajectory optimization
% create initial guess
initial_guess = zeros(2,1) ;

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
            
tic
% call fmincon
[k_opt,~,exitflag] = fmincon(cost,...
                            initial_guess,...
                            [],[],... % linear inequality constraints
                            [],[],... % linear equality constraints
                            k_bounds(:,1),... % lower bounds
                            k_bounds(:,2),... % upper bounds
                            nonlcon,...
                            options) ;
toc                     
% check the exitflag
if exitflag <= 0
    k_opt = [] ;
end

k_scale = [1 .34];

U0 = k_opt(1)*k_scale(1) + 1;
U1 = k_opt(2)*k_scale(2);
disp(U0),disp(U1)


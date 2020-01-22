%U[0] longitduinal velocity
%U[1] steering angle

%        relation k to U
%        U[0] = k[0]*k_scale[0]+k_const
%        U[1] = k[1]*k_scale[1]
%

%        dynamics in U 
%        dX[0] = U[0]-(w*X[1])
%        dX[1] = w*X[0]

%        X[0] = x pos, X[1] = y pos
%        where w = yaw rate = U[0]*U[1]/wb
%        and wb = wheel base


%% User parameters
% degree of SOS polynomial solution

degree = 10 ; % this should be 4 or 6 unless you have like 100+ GB of RAM
formatlab = 0; % DEPENDING ON SOLVER, RHS inequality is either 1 or 0.
load('turtlebot_error_functions_v0_0.0_to_0.5.mat'); % Approximate
load('rover_timing.mat'); 

% Rover footprint is a box
rover_length = .55 ; 
wb = .32; % wheelbase, measured
rover_width = .3; 

k_scale = [1 .34];
k_const = 1;

%% Time scale for FRS computation

time_scale = t_f ; % Compute FRS without considering braking, thus use t_f.

%% Compute distance scale

T = [0 1] ;
U = [2 0] ; % 2m/s speed, strait ahead. this will choose dx as distance scale.
z0 = zeros(2,1); 
[~,Z] = ode45(@(t,z) rover_trajectory_producing_model(t,z,U,1,1),[0 t_f],z0) ;
dx = max(Z(:,1)) - min(Z(:,1)) + rover_length ;
dy = max(Z(:,2)) - min(Z(:,2)) + rover_width ;
distance_scale = max(dx,dy) ;
initial_x =  -0.15 ;
initial_y =  0.0 ;


%% set up the FRS computation variables and dynamics

% set up the indeterminates
t = msspoly('t', 1) ; % time t \in T
z = msspoly('z', 2) ; % state z = (x,y) \in Z
k = msspoly('k', 2) ; % parameters k \in K

x = z(1) ; y = z(2) ;

% create polynomials that are positive on Z, and K, thereby
% defining them as semi-algebraic sets
Z_range = [-1, 1 ; -1, 1] ; % z \in [-1,1]^2

K_range = [-1, 1 ; -1, 1] ; % k \in [-1,1]^2

h_Z = (z - Z_range(:,1)).*(Z_range(:,2) - z) ;

scl_x = (rover_length/2)/distance_scale;
scl_y = (rover_width/2)/distance_scale;
% h_Z0 = 1 - ((x - initial_x)/(scl_x)).^2 + ...
%          - ((y - initial_y)/(scl_y)).^2 ;
     
h_Z0 = [(x-initial_x)+scl_x ;... 
        -(x-initial_x)+scl_x ;...   
        (y-initial_y)+scl_y;...
        -(y-initial_y)+scl_y]; 
% z0 positive on box footprint, centered at initial position- approximated by ellipse 

h_K = (k - K_range(:,1)).*(K_range(:,2) - k) ;


%% Specify dynamics and error function

U0 = k(1)*k_scale(1) + k_const;
U1 = k(2)*k_scale(2);

% create dynamics
scale = (time_scale/distance_scale) ;
w = distance_scale*U0*U1/wb;
f = scale*[ U0 - w*(y - initial_y) ;
                w*(x - initial_x)] ;

% create tracking error dynamics; first, make the monomials of time in
% decreasing power order
g_x_t_vec = t.^(length(g_x_coeffs)-1:-1:0) ;
g_y_t_vec = t.^(length(g_y_coeffs)-1:-1:0) ;
g = 0.3*scale*[g_x_t_vec*g_x_coeffs', 0 ;
           0, g_y_t_vec*g_y_coeffs'] ;
       
%% Create cost function
% this time around, we care about the indicator function being on Z x K
int_ZK = boxMoments([z;k], [Z_range(:,1);K_range(:,1)], [Z_range(:,2);K_range(:,2)]);

%% Setup the problem structure
solver_input_problem.t = t ;
solver_input_problem.z = z ;
solver_input_problem.k = k ;
solver_input_problem.f = f ;
%solver_input_problem.g = g ; 
solver_input_problem.hZ = h_Z ;
solver_input_problem.hZ0 = h_Z0 ;
solver_input_problem.hK = h_K ;
solver_input_problem.cost = int_ZK ;
solver_input_problem.degree = degree ;

%% Compute FRS 
solve_time = tic ;
solver_output = compute_FRS(solver_input_problem) ;
solve_time = toc(solve_time) ;

%% Extract FRS polynomial result/ Post processing

FRS_polynomial = solver_output.indicator_function ;
FRS_lyapunov_function = solver_output.lyapunov_function ;

% save!
    save(['mat_files/rover_FRS_prestruct_deg_',num2str(degree)],'FRS_polynomial',...
        'FRS_lyapunov_function','t','z','k',...
        'time_scale','distance_scale','initial_x','initial_y',...
        'rover_width','rover_length','f','g','initial_x','initial_y','t_f',...
        't_plan','t_stop','degree','h_Z','h_Z0','h_K','k_scale','k_const','wb')
    
FRS = load(['mat_files/rover_FRS_prestruct_deg_',num2str(degree)]);

% get FRS polynomial and variables
if formatlab
    FRS_msspoly = FRS.FRS_polynomial-1;
else
    FRS_msspoly = FRS.FRS_polynomial;
end
k = FRS.k ;
z = FRS.z ;

% decompose polynomial into simplified structure used online
FRS_poly = get_FRS_polynomial_structure(FRS_msspoly,z,k) ;

if formatlab
    save(['rover_FRS_deg_',num2str(degree),'_matlab'],'FRS_poly');
else
    FRS_poly.coef = FRS_poly.coef';
    FRS_poly.dist_scale = distance_scale;
    FRS_poly.k_scale = k_scale;
    FRS_poly.k_const = k_const;
    FRS_poly.t_f = t_f;
    FRS_poly.t_plan = t_plan;
    FRS_poly.t_stop = t_stop;
    FRS_poly.T = T(end);
    FRS_poly.footprint_W = rover_width;
    FRS_poly.wb = wb;
    FRS_poly.degree = degree;
    FRS_poly.initial_x = initial_x;
    FRS_poly.initial_y = initial_y;
    FRS_poly.box_FRS = [-0.45 0.80 -0.85 0.85]
    clearvars -except FRS_poly
    pows = full(FRS_poly.pows);
    coef = full(FRS_poly.coef);
    z_cols = FRS_poly.z_cols-1;
    k_cols = FRS_poly.k_cols-1;
    t_cols = FRS_poly.t_cols;
    dist_scale = FRS_poly.dist_scale;
    k_scale = FRS_poly.k_scale;
    k_const = FRS_poly.k_const;
    t_f = FRS_poly.t_f;
    t_plan = FRS_poly.t_plan;
    t_stop = FRS_poly.t_stop;
    T = FRS_poly.T;
    footprint_W = FRS_poly.footprint_W;
    wb = FRS_poly.wb;
    degree = FRS_poly.degree;
    initial_x = FRS_poly.initial_x;
    initial_y = FRS_poly.initial_y;
    box_FRS = FRS_poly.box_FRS;
    clear FRS_poly;

    save(['rover_FRS_deg_',num2str(degree)])
end

       


  




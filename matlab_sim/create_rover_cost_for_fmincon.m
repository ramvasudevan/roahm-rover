%create_rover_cost_for_fmincon

syms k_0 k_1 Zg_0 Zg_1
X_0 = 0;
X_1 = 0;
k_scl = [1 .34]; % match from FRS mat file
wb = 0.32; % from FRS mat file
h = 0.125; % match to python file
t_f = 1.17; % from FRS mat file
distance_scale = 2.89; % from FRS mat file
dyn_scl = h*(t_f/distance_scale); % from FRS file mat t_f/dist_scale

n = 1/h;
for k = 1:n
    w = (distance_scale*(k_scl(1)*k_0+1)*(k_scl(2)*k_1)/wb);
    dX_0 = (k_scl(1)*k_0+1)-(w*X_1);
    dX_1 = w*X_0;
    X_0 = X_0+dyn_scl*dX_0;
	X_1 = X_1+dyn_scl*dX_1;
end
expr = (X_0-Zg_0)^2+(X_1-Zg_1)^2;
rover_cost = matlabFunction(expr,'Vars',[k_0,k_1,Zg_0,Zg_1],'File','rover_cost');

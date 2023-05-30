clc;
clear all;

% Initialize the environment and problem parameters
R = ；
eta = 1;
rho = 1000;
A = pi * R^2;
P_cap = ;
J = ;
lambda_star = ;
M = ;

%coeffiecients for mid-range spring damper 
K_surge = 800e3;
D_surge = 800e3;

%coeffiecients for stiff spring damper 
%K_surge = 1200e3;
%D_surge = 2500e3;

%coeffiecients for flexible spring damper 
%K_surge = 100e3;
%D_surge = 180e3;

%preprosss data, cp, thrust, and drag.
path = 'hktdata_cp.txt';
cp_array = readmatrix(path);
cp_array = cp_array(:, 2:end);

path = 'hktdata_thrust.txt';
Ft_array = readmatrix(path);
Ft_array = Ft_array(:, 2:end);

% Process drag data and approximate Cd by taking the average
path = 'drag.txt';
Drag_array = readmatrix(path);
Drag_v = Drag_array(:, 1);
Drag_f = Drag_array(:, 2);
Cd = mean(Drag_f ./ (rho .* Drag_v.^2 .* A / 2));


% Helper Function to obtain the closest tsr_idx given the value of tsr
get_tsr_idx = @(tsr) min(max(floor(tsr * 10) - 39, 1), 61);



% Solve the ODE

tspan = [0:1:120]'; %time span
t = [0:1:120]'; 
y0 = [1, 0.05, 0.5]; %initial y0

theta_list = [];
x1_list = [];
x2_list = [];
omega_list = [];
tsr_list = [];
P_list = [];



for i = 0:0   %Solve for a series of v, and solve only 1 set of v0 by default;
v0 = 3 + i*1;
v = v0 + 0.25 * sin(t / 5);
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);
[t,result_odeint] = ode45(@(t, y) deriv_base(t, y, v0, cp_array, Ft_array, R, Cd, rho, A, J, eta, lambda_star, K_surge, D_surge,get_tsr_idx,M), tspan, y0, options);


for k = 1:numel(t)
    [~,theta(k,:),cp(k,:)] = deriv_base(t(k),result_odeint(k,:),v0,cp_array, Ft_array, R, Cd, rho, A, J, eta, lambda_star, K_surge, D_surge,get_tsr_idx,M);
end

%getting results
x1_array = result_odeint(:, 1);
x2_array = result_odeint(:, 2);
omega_array = result_odeint(:, 3);
tsr_array = omega_array*R./(v - x2_array);
P = 0.5*rho*A.*cp.*(v - x2_array).^3;

%concatenate list if solve for more than 1 set of water velocity v
theta_list = [theta_list,theta];
x1_list = [x1_list,x1_array];
x2_list = [x2_list,x2_array];
omega_list = [omega_list,omega_array];
tsr_list = [tsr_list,tsr_array];
P_list = [P_list,P];
end

%plotting figures

%plot rotational velocity, water velocity and tsr
fig1=figure(1);
plot(t,omega_array,'g');
hold on
plot(t,v,'r');
xlabel('time (s)')
hold on
yyaxis right
plot(t, tsr_array,'b');
hold on
legend ('rotational velocity (rad/s)','water velocity (m/s)','tsr', 'Location','southeast')
title("v = " + v0 + "+ 0.25*sin(t/5)")
%saveas(fig1,'tsr.png');

%plot surge motion
fig2=figure(2);
plot(t,v,'r');
hold on
plot(t,x1_array)
xlabel('time (s)')
hold on
plot(t,x2_array,'-g')
hold on
legend ('water velocity (m/s)','displacement (m)', 'mass velocity (m/s)', 'Location', 'southeast')
title("Surge motion for v = " + v0 + " + 0.25*sin(t/5)")
%saveas(fig2,'surge_flexi.png')

%plot power
fig3=figure(3);
plot(t,P_list(:,1), 'b','LineWidth',3)
hold on
xlabel('time (s)')
hold on
ylabel('power (W)')
yline(160000,'-','Threshold');
ylim([0,2.0e5])
%hold on
legend('power')
title("Power for v = 3+0.25sin(t/5)")
%saveas(fig3,'power.png');

%plot pitch angle
fig4 = figure(4);
plot(t,theta_list(:,1), 'r','LineWidth',2)
hold on
xlabel('time (s)')
ylabel('pitch angle (deg)')
%saveas(fig4, 'pitch_angle.png')

fig5 = figure(5);




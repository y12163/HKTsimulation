% Function to compute derivatives of the system

%input: environment parameters
%       time span t
%       initial state y0
%               state(1), displacement of the system
%               state(2), velocity of the system
%               state(3), rotor rotational velocity omega

%output: evluated state at each time step
%        pitch angle theta at each time step
%        cp at each time step

function [dydt,theta,cp] = deriv_base(t, state, v0,cp_array, Ft_array, R, Cd, rho, A, J, eta, lambda_star, K_surge, D_surge,get_tsr_idx,M)
    v = v0 + 0.25 * sin(t / 5);
    vrel = v - state(2);
    tsr = state(3) * R / vrel;
    tsr_idx = get_tsr_idx(tsr);
    cp = max(cp_array(tsr_idx, :));   

    %apply power constraint using pitch control
    P_cap = 160000;
    cp_cap = P_cap/(0.5*rho*A*vrel^3);
    cp = min(cp,cp_cap);  
    theta = angle_interp(tsr_idx,cp,cp_array);                %pitch angle

    F_t = thrust_interp(tsr_idx,theta,Ft_array)*1000;         %thrust force
    F_d = Cd * rho * (v - state(2))^2 * A / 2;                %drag force
    M_r = 0.5 * rho * pi * R^3 * cp / tsr * (v - state(2))^2; %rotor torque
    K = 0.5 * rho * A * R^3 * 0.4714 / lambda_star^3;         
    M_g = K * state(3)^2;                                     %generator torque

    dydt = [
        state(2);                                             %xdot
        (F_t + F_d - K_surge * state(1) - D_surge * state(2)) / M;  %vdot
        1 / J * (M_r - M_g / eta)                             %omegadot
    ];
end


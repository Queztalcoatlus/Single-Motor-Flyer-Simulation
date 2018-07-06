clear all; close all; clc;
%initial states
input = 0;
theta = zeros(3,1);%row, pitch and yaw
thetadot = zeros(3,1);
xdot = zeros(3,1);
x = zeros(3,1);

%parameters
g = [0; 0; -9.8];
m = 0.5;
d = [0,0,0.17];

I_B_B = diag([2.7*10e-3, 2.7*10e-3,5.2*10e-3]);  % Body Inertia
I_B_P = diag([0, 0,1.5*10e-5]);  % Propeller Inertia

dt=0.1;%simulation time interval in seconds

%aerodynamic drag torque
K_B_d=diag([0.7*10e-4, 0.7*10e-4, 1.4*10e-4]);

% Compute thrust given current inputs and thrust coefficient.
f = [0; 0; 6.41*10^(-6) * input*input];

% Compute reaction torque, given current inputs, length, drag coefficient, and thrust coefficient.
T_p = -input*input* 1.1 * 10e-7;

% Compute forces, torques, and accelerations.
omega = thetadot2omega(thetadot, theta);
%acceleration in earth frame
R = rotation(theta);%rotation matrix from body frame to earth frame
T = R * f;
a = g + 1 / m * T;
%omegadot
t_B_d=-omega*K_B_d*omega;
tau=cross(d',f)+T_p;%total torque

omegadot = I_B_B\ (tau + t_B_d - cross(omega, I_B_B * omega+I_B_P*(R*input)));

% Advance system state.
omega = omega + dt * omegadot;
thetadot = omega2thetadot(omega, theta); 
theta = theta + dt * thetadot;
d_dot = d_dot + dt * a;
d = d + dt * d_dot;

% Convert derivatives of roll, pitch, yaw to omega.
function omega = thetadot2omega(thetadot, angles)
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
    W = [
        1, 0, -sin(theta)
        0, cos(phi), cos(theta)*sin(phi)
        0, -sin(phi), cos(theta)*cos(phi)
    ];
    omega = W * thetadot;
end

% Convert omega to roll, pitch, yaw derivatives
function thetadot = omega2thetadot(omega, angles)
    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
    W = [
        1, 0, -sin(theta)
        0, cos(phi), cos(theta)*sin(phi)
        0, -sin(phi), cos(theta)*cos(phi)
    ];
    thetadot = inv(W) * omega;
end

function R = rotation(angles)
    phi = angles(3);
    theta = angles(2);
    psi = angles(1);

    R = zeros(3);
    R(:, 1) = [
        cos(phi) * cos(theta)
        cos(theta) * sin(phi)
        - sin(theta)
    ];
    R(:, 2) = [
        cos(phi) * sin(theta) * sin(psi) - cos(psi) * sin(phi)
        cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi)
        cos(theta) * sin(psi)
    ];
    R(:, 3) = [
        sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta)
        cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi)
        cos(theta) * cos(psi)
    ];
end

%function [f_last,x,x_dot,R,theta,omega,counter] = fcn(f_old,f,x_old,x_dot_old,theta_old,omega_old,counter_old)
% the block is based on the dynamics of a quadcopter on
% http://andrew.gibiansky.com/blog/physics/quadcopter-dynamics/
clear;
ts = 0;
tend = 10;
dt = 0.001;
n = (tend - ts)/dt;
xout = zeros(3,n);
xdotout = zeros(3,n);
thetaout = zeros(3,n);
thetadotout = zeros(3,n);
omegaout = zeros(3,n);
omegadotout = zeros(3,n);
counter = 0;
x = zeros(3,1);
x_dot = zeros(3,1);
theta = zeros(3,1);
thetadot = zeros(3,1);
omega = zeros(3,1);
for i=0:0.001:10
    counter=counter+1;
    kd=0.25;

    %parameters
    g = [0; 0; -9.8];
    m = 0.217;
    d = [1,0,0];

    %I_B_B = [102 24 9;24 318 0;9 0 414]*10^(-5);  % Body Inertia
    I_B_B = diag([5e-3, 5e-3, 10e-3]);
    I_B_P = diag([0, 0,1.5e-5]);  % Propeller Inertia
    %thrust and torque coefficient
    K_f=6.41*10^(-6);
    K_t=1.1 * 10^(-7);
    dt=0.001;%simulation time interval in seconds

    %aerodynamic drag torque
    K_B_d=diag([0.7e-4, 0.7e-4, 1.4e-4]);
    
    %R = rotation([0;0;0]);
    R = rotation(theta);
    %R = eul2rotm(theta','ZYX');
    % Compute reaction torque, given current inputs, length, drag coefficient, and thrust coefficient.
    f = 0.001;
    f_vector=[0; 0; f];
    T = R * f_vector;
    a = 1 / m .* T;
    
    omegadot = I_B_B\(cross(d',f_vector)+[0;0;0.01] - cross(omega, I_B_B * omega));
    omega = omega + dt * (omegadot);
    %omega=[0;0;0];
    thetadot = omega2thetadot(omega, theta); 
    theta = theta + dt * thetadot;
    x_dot = x_dot + a * dt;
    %x_dot=[0;0;0];
    x = x + dt * x_dot;
    %x=[0;0;0];
    %testing
    if counter==100
        counter=100;
    end
    % Store simulation state for output.
        xout(:, counter) = x;
        xdotout(:, counter) = x_dot;
        thetaout(:, counter) = theta;
        thetadotout(:,counter) = thetadot;
        omegaout(:,counter)=omega;
        omegadotout(:,counter)=omegadot;
end
figure(1)
plot(linspace(0,10,n),xout(1,1:10000),linspace(0,10,n),xout(2,1:10000),linspace(0,10,n),xout(3,1:10000));
legend('x','y','z');
xlabel('time (s)');
ylabel('position (m)');

figure(2)
plot(linspace(0,10,n),xdotout(1,1:10000),linspace(0,10,n),xdotout(2,1:10000),linspace(0,10,n),xdotout(3,1:10000));
legend('x','y','z');
xlabel('time (s)');
ylabel('Velocity (m/s)');

%Angular Velocity Omega in the body frame
figure(3)
plot(linspace(0,10,n),omegaout(1,1:10000),linspace(0,10,n),omegaout(2,1:10000),linspace(0,10,n),omegaout(3,1:10000));
legend('x','y','z');
xlabel('time (s)');
ylabel('omega (rad/s)');

figure(4)
plot(linspace(0,10,n),omegadotout(1,1:10000),linspace(0,10,n),omegadotout(2,1:10000),linspace(0,10,n),omegadotout(3,1:10000));
legend('x','y','z');
xlabel('time (s)');
ylabel('omegaot (rad/s^2)');

figure(5)
plot(linspace(0,10,n),mod(thetaout(1,1:10000),2*pi),linspace(0,10,n),mod(thetaout(2,1:10000),2*pi),linspace(0,10,n),mod(thetaout(3,1:10000),2*pi));
legend('x','y','z');
xlabel('time (s)');
ylabel('theta (rad)');

figure(6)
plot(linspace(0,10,n),thetadotout(1,1:10000),linspace(0,10,n),thetadotout(2,1:10000),linspace(0,10,n),thetadotout(3,1:10000));
legend('x','y','z');
xlabel('time (s)');
ylabel('thetadot (rad/s)');
f_last
function [f_last,x,x_dot,R,theta,omega,counter] = fcn(f_old,f,x_old,x_dot_old,theta_old,omega_old,counter_old)
% the block is based on the dynamics of a quadcopter on
% http://andrew.gibiansky.com/blog/physics/quadcopter-dynamics/
    counter=counter_old+1;
    if counter==15
        counter=15;
    end
%Assume a friction coeffiicent
    kd=0.25;
    %initial states
    %row, pitch and yaw
    theta = zeros(3,1);
    thetadot = zeros(3,1);

    %parameters
    g = [0; 0; -9.8];
    m = 0.5;
    d = [0,0,0.17];

    I_B_B = diag([2.7*10e-3, 2.7*10e-3,5.2*10e-3]);  % Body Inertia
    I_B_P = diag([0, 0,1.5*10e-5]);  % Propeller Inertia

    dt=0.01;%simulation time interval in seconds

    %aerodynamic drag torque
    K_B_d=diag([0.7*10e-4, 0.7*10e-4, 1.4*10e-4]);

    % Compute thrust given current inputs and thrust coefficient.
    %f = [0; 0; 6.41*10^(-6) * input*input];
    prop_speed_sq=f/(6.41*10^(-6));
    prop_speed=[0;0;sqrt(abs(prop_speed_sq))];
    prop_speed_sq_old=f_old/(6.41*10^(-6));
    prop_speed_old=[0;0;sqrt(abs(prop_speed_sq_old))];
    % Compute reaction torque, given current inputs, length, drag coefficient, and thrust coefficient.
    T_p = -prop_speed_sq* 1.1 * 10e-7;
    f_vector=[0; 0; -f];
    % Compute forces, torques, and accelerations.
    %omega = thetadot2omega(thetadot, theta);
    %This seems to be wrong
    %acceleration in earth frame
    R = rotation(theta_old);%rotation matrix from body frame to earth frame
    %R=zeros(3,3);
    T = R * f_vector;
    a = g + 1 / m .* T-kd*x_dot_old.^2;
    %omegadot
    t_B_d=-sqrt(sum((prop_speed).^2))*K_B_d*prop_speed;
    tau=cross(d',f_vector)+T_p;%total torque
    prop_ang_acc=(prop_speed-prop_speed_old)/dt;
    omegadot = I_B_B\ (tau + t_B_d - cross(omega_old, I_B_B * omega_old+I_B_P*(R*prop_speed))-I_B_P*(R*prop_ang_acc));
    if max(omegadot)>10^20
        omegadot=[0;0;0];
    end
    % Advance system state
    omega = omega_old + dt .* omegadot;
    %omega=[0;0;0];
    thetadot = omega2thetadot(omega, theta); 
    theta = theta_old + dt * thetadot;
    x_dot = x_dot_old + a * dt;
    %x_dot=[0;0;0];
    x = x_old + dt * x_dot;
    f_last=f;
    %x=[0;0;0];
    %SetOutputPortDimensions(s, 2, [3 1]);
end



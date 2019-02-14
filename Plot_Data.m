%Run after Single_motor_flyer_controller.slx
%Position
figure(1)
pos = yout{1}.Values.Data;
pos_x = squeeze(pos(1,1,:));
pos_y = squeeze(pos(2,1,:));
pos_z = squeeze(pos(3,1,:));
plot(tout,pos_x,tout,pos_y,tout,pos_z);
legend('x','y','z');
xlabel('time (s)');
ylabel('position (m)');

%Velocity
figure(2)
v = yout{2}.Values.Data;
v_x = squeeze(v(1,1,:));
v_y = squeeze(v(2,1,:));
v_z = squeeze(v(3,1,:));
plot(tout,v_x,tout,v_y,tout,v_z);
legend('x','y','z');
xlabel('time (s)');
ylabel('velocity (m/s)');

%Angular Velocity Omega in the body frame
figure(3)
omega = yout{3}.Values.Data;
omega_x = squeeze(omega(1,1,:));
omega_y = squeeze(omega(2,1,:));
omega_z = squeeze(omega(3,1,:));
plot(tout,omega_x,tout,omega_y,tout,omega_z);
legend('x','y','z');
xlabel('time (s)');
ylabel('omega (rad/s)');

%Angular Velocity Omega in the body frame
figure(4)
theta = yout{4}.Values.Data;
theta_x = squeeze(theta(1,1,:));
theta_y = squeeze(theta(2,1,:));
theta_z = squeeze(theta(3,1,:));
plot(tout,theta_x,tout,theta_y,tout,theta_z);
legend('x','y','z');
xlabel('time (s)');
ylabel('theta (rad)');

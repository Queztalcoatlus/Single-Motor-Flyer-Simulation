%The caller calls the function of the flyer block
clear;clc;
%[x,x_dot,R,theta,omega] = fcn(f,x_old,x_dot_old,theta_old,omega_old)
[x,x_dot,R,theta,omega] = dynamics_test(1,[1;1;1],[1;1;1],[1;1;1],[1;1;1]);
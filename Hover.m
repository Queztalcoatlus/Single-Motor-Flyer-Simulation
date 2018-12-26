clear;clc;

g=9.8;

%Quadcopter Case
% m=0.5;
% eB_P=[0 0 1];
% IB_B=diag([2.7 2.7 5.2])*10^(-3);
% IB_B=[102 24 9;24 318 0;9 0 414]*10^(-5);
% IB_P=diag([0 0 1.5])*10^(-5);
% rB_P=[0.17;0;0];
% KB_d=diag([0.7 0.7 1.4])*10^(-4);
% K_tau=1.1*10^(-7);
% K_f=-6.41*10^(-6);

%Monospinner
m=0.217;
eB_P=[0 0 1];
IB_B=[102 24 9;24 318 0;9 0 414]*10^(-5);
IB_P=diag([0 0 1.5])*10^(-5);
rB_P=[0.17;0;0];
KB_d=diag([0.55 0.55 1.1])*10^(-4);
K_tau=1.1*10^(-7);
K_f=6.41*10^(-6);
%Hover equations
syms a1 a2 a3 OMG
omgB_BE=[a1;a2;a3];
omgB_B_PB=[0;0;OMG];
omgB_PE=omgB_BE+omgB_B_PB;
%omgB_BE=sym('omgB_BE',[1 3]);
%omgB_PE=sym('omgB_PE',[1 3]);

tau_P=-K_tau*dot(omgB_PE,eB_P).*abs(dot(omgB_PE,eB_P));

tauB_d=-sqrt(dot(omgB_BE,omgB_BE)).*(KB_d*omgB_BE);

fp_bar=m*g*sqrt(sum((omgB_BE).^2))/sqrt(sum((eB_P*omgB_BE).^2));

fp=K_f*dot(omgB_PE,eB_P).*abs(dot(omgB_PE,eB_P));

eqn1=fp-fp_bar;
eqn2=cross(omgB_BE,(IB_B*omgB_BE+IB_P*omgB_PE))-cross(rB_P,eB_P)'.*fp_bar-(eB_P*tau_P)'-tauB_d;

sol=solve([eqn1,eqn2(1),eqn2(2),eqn2(3)],[a1,a2,a3,OMG]);

ans_omgB_BE=[double(sol.a1);double(sol.a2);double(sol.a3)];
ans_OMG=double(sol.OMG);
ans_fp=m*g*sqrt(sum((ans_omgB_BE).^2))/sqrt(sum((eB_P*ans_omgB_BE).^2));

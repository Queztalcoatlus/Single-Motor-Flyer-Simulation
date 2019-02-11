%function u = linearization(n_des_C,omega_C_BE,fp,f_pos)
    %Hover_solution
    f_pos=5;
    omega_hover=[7.0;-3.7;22.4];
    %C_CB=[0;0;sqrt(sum(omega_hover.^2))]/omega_hover;
    euler = [atan(omega_hover(2)/omega_hover(1)),acos(omega_hover(3)/sqrt(sum(omega_hover.^2))),0];
    C_CB = inv(eul2rotm(euler,'ZYX'));
    fp_hover=2.26;
    %f_pos should not be one of the inputs
    %parameters
    g = [0; 0; -9.8];
    m = 0.5;
    d = [0.17,0,0];
    %Moment of Inertia
    %I_B_B = diag([2.7e-3, 2.7e-3,5.2e-3]);  % Body Inertia
    I_B_B = [102, 24, 9; 24, 318, 0; 9 0 414]*10^-(5);
    I_B_P = diag([0, 0,1.5e-5]);  % Propeller Inertia
    %thrust and torque coefficient
    K_f=6.41*10^(-6);
    K_t=1.1 * 10^(-7);
    dt=0.015;%time constant
    %drag coefficient
    K_B_d=diag([0.7e-4, 0.7e-4, 1.4e-4]);
    %S=[n_des_C(1,1),;n_des_C(2,1);omega_C_BE(1,1);omega_C_BE(2,1);omega_C_BE(3,1)];
    S=sym('S',[6 1]); %state
%     U=sym('U',[6 1]);
%     U(1:5,1)=[0;0;0;0;0];
    %syms u_sym %input
    % n_des_C_sym=zeros(3,1);
    n_des_C_sym=sym('N',[3 1]);
    n_des_C_sym(1:2,1)=S(1:2,1);
    n_des_C_sym(3,1)=sqrt(1-S(1,1)^2-S(2,1)^2);
    %In control coordinate C
    
    omega_sym = pinv(C_CB)*(S(3:5,1)-C_CB*omega_hover); %Changed
    
    fp_sym=S(6,1)+fp_hover;
    n_des_C_dot=-cross(C_CB*omega_sym,n_des_C_sym);
    F=sym('F',[6 1]); %input
    fp_dot=(f_pos+F(6,1)-fp_sym)/dt;
    
    
    %disregard drag and angular acceleration of the propeller, and
    %represent the thrust in the inertial frame in order to calculate the
    %angular acceleration of body in the inertial frame
    prop_speed_sq=fp_sym/K_f;
    prop_speed=[0;0;sqrt(abs(prop_speed_sq))]-[0;0;omega_sym(3,1)];
    %From fp_dot=2*K_f*prop_speed_z*prop_speed_z_dot  (prop_speed_z is the scalar value of prop_speed)
    prop_ang_acc=[0;0;fp_dot/(2*K_f*sqrt(prop_speed_sq))]; 
    t_B_d=-sqrt(sum((omega_sym).^2))*K_B_d*omega_sym;
    T_p = [0;0;prop_speed_sq* K_t];
    f_vector=[0; 0; fp_sym];
    tau=(cross(d',f_vector)+T_p) + t_B_d;%total torque
    % omega_B_BE_dot
    omegadot = I_B_B\(tau - cross(omega_sym, I_B_B * omega_sym+I_B_P * omega_sym+I_B_P*(prop_speed))-I_B_P*(prop_ang_acc));
    %omegadot = I_B_B\(tau + t_B_d - cross(omega_sym, I_B_B * omega_sym));
    %S_dot
    S_dot=[n_des_C_dot(1:2,1);omegadot;fp_dot];
    %Jacobian
    %F=[0;0;0;0;0;u_sym];
    
    %F=[0;0;0;0;0;u_sym];
    A=jacobian(S_dot,S);
    B=jacobian(S_dot,F(6,1));
    B_1=subs(B,F,[0;0;0;0;0;0]);
    B_2=subs(B_1,S,zeros(6,1));
    B_res=double(B_2);
    A_1=subs(A,F,[0;0;0;0;sqrt(sum(omega_hover.^2));fp_hover]);
    A_2=subs(A_1,S,zeros(6,1));
    A_res=double(A_2);
    %LQR
    Q=diag([75;75;0;0;0;0]);
%     R=zeros(6,6);
%     R(6,6)=1;
    R=1;
    N=zeros(6,1);
    [K,S_lqr,e] = lqr(A_res,B_res,Q,R,N);
%     s=zeros(6,1);
%     s(1:2,1)=n_des_C(1:2,1);
%     s(3:5,1)=omega_C_BE+omega_hover;
%     s(6,1)=fp-fp_hover;
%     u = -K*s;
%end
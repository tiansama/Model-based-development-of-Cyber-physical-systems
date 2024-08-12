%% the whole model 
clc
clear all
%% load the mat
load("FlightData.mat")
%% parameters for complementary filter
gamma = 0.1;
h = 0.01;
alpha = (gamma * h) / (1 - gamma); 

syms c1 c2 c3 c4 theta phi theta_d phi_d psi_d
%% other parameters
m = 0.027;                  % Mass
g = 9.81;                   % Gravitational acceleration
d = 0.046;                  % Distance from center of mass to rotor axis
lift = 1.9796E-9;           % Lift constant
drag =  2.5383E-11;         % Drag constant
%% Inertia
Jx = 1.1463e-5;
Jy = 1.6993e-5;
Jz = 2.9944e-5;

J = [Jx,0,0;0,Jy,0;0,0,Jz];
%% torque
% Torque
ctrl = [c1;c2;c3;c4]; 
r=sqrt((ctrl)/lift);
Torque_x=d*lift*(sqrt(2)/2)*(r(3)^2+r(4)^2-r(1)^2-r(2)^2);
Torque_y=d*lift*(sqrt(2)/2)*(r(2)^2+r(3)^2-r(1)^2-r(4)^2);
Torque_z=drag*(-r(1)^2-r(3)^2+r(2)^2+r(4)^2);
Torque=[Torque_x,Torque_y,Torque_z]';

x = [phi;phi_d;theta;theta_d;psi_d];
y = x;

W=[1 tan(theta)*sin(phi) -tan(theta)*cos(phi);
   0    cos(phi)            sin(phi);
   0    -sin(phi)/cos(theta) cos(phi)/cos(theta)];
w = W\[phi_d;theta_d;psi_d];
w_d = J\(-cross(w,J*w)+Torque);

x_d = [w(1);w_d(1);w(2);w_d(2);w_d(3)];

% Steady-state operating point.
theta0=0;                    
phi0=0;     
theta_d0=0;                     
phi_d0=0;
psi_d0=0;
x0 = [phi0;phi_d0;theta0;theta_d0;psi_d0];

% Determine the full operating point.
x_dot_subs = subs(x_d, x, x0);
ctrl_sol = solve(x_dot_subs == 0, ctrl);
c10 = double(vpa(ctrl_sol.c1, 4));
c20 = double(vpa(ctrl_sol.c2, 4));
c30 = double(vpa(ctrl_sol.c3, 4));
c40 = double(vpa(ctrl_sol.c4, 4));
ctrl0 = [c10; c20; c30; c40];

% Calculate symbolic Jacobians.
A_sym = jacobian(x_d, x);
B_sym = jacobian(x_d, ctrl);
C_sym = jacobian(y, x);
D_sym = jacobian(y, ctrl);

% Linearize around given operating point
A_lin = double(subs(A_sym,[ctrl; x],[ctrl0; x0]));
B_lin = double(subs(B_sym,[ctrl; x],[ctrl0; x0]))
C_lin = double(subs(C_sym,[ctrl; x],[ctrl0; x0]));
D_lin = double(subs(D_sym,[ctrl; x],[ctrl0; x0]));




sys=ss(A_lin,B_lin,C_lin,D_lin);
% Discretize 
h=0.01;
sys_d=c2d(sys,h);
A_d=sys_d.a;
B_d=sys_d.b;
C_d=sys_d.c;
D_d=sys_d.d;


% Design LQR controller
Q_x=eye(5);
Q_u=eye(4);
Q_x(1,1)=2;
Q_x(3,3)=2;



L = dlqr(A_d,B_d,Q_x,Q_u)
% K=pinv(C_d*inv(eye(5)-A_d+B_d*L)*B_d)
closed_loop
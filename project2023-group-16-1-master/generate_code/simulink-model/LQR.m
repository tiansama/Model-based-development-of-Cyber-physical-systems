%
% LQR
%

% clc;
% clear;

%Continiues time
A = [0, 1, 0, 0, 0;
     0, 0, 0, 0, 0;
     0, 0, 0, 1, 0;
     0, 0, 0, 0, 0;
     0, 0, 0, 0, 0];

B =[        0,        0,         0,        0;
    -0.33703, -0.33703,   0.33703,  0.33703;
            0,        0,         0,        0;
    -0.22735,  0.22735,   0.22735, -0.22735;
    -0.050861, 0.050861, -0.050861, 0.050861];

C = [eye(4) zeros(4,1)]
    
D = 0


sys = ss(A,B,C,D)
%Discretize
Ts = 0.01;
sys_d = c2d(sys,Ts);
Ad = sys_d.A;
Bd = sys_d.B;
Cd = sys_d.C;
Dd = sys_d.D;

Qx = eye(5);
Qu = eye(4);


[Kd,S,e] = lqrd(Ad,Bd,Q,R,Ts)
% G = inv(C*(A-B*K)*B) %look at lecture notes


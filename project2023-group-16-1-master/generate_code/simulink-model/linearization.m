clear

syms phi dphi ddphi theta dtheta ddtheta dpsi ddpsi m g k b d u1 u2 u3 u4 Jx Jy Jz real

% WRB only relevant for orientation estimate, not plant
% Rx = @(phi) [
%     1           0           0
%     0           cos(phi)    -sin(phi)
%     0           sin(phi)    cos(phi)
% ];
% Ry = @(theta) [
%     cos(theta)  0           sin(theta)
%     0           1           0
%     -sin(theta)  0           cos(theta)
% ];
% Rz = @(psi) [
%     cos(psi)    -sin(psi)   0
%     sin(psi)    cos(psi)    0
%     0           0           1
% ];
% BRWxyz = @(phi, theta, psi) (Rz(psi) * Ry(theta) * Rx(phi)); 

% gyroscope omega from body to world
BRWxyz_g = @(phi, theta, psi) [
    1           0           sin(theta)
    0           cos(phi)    -sin(phi)*cos(theta)
    0           sin(phi)    cos(phi)*cos(theta)
];

x = [phi dphi theta dtheta dpsi]';
u = [u1 u2 u3 u4]';
omega_motor = sqrt(u / b); % to not confuse with rate of orientation change omega

Tx = d * sin(pi / 4) * b * (-omega_motor(1)^2 - omega_motor(2)^2 + omega_motor(3)^2 + omega_motor(4)^2);
Ty = d * sin(pi / 4) * b * (-omega_motor(1)^2 + omega_motor(2)^2 + omega_motor(3)^2 - omega_motor(4)^2);
Tz = k * (-omega_motor(1)^2 + omega_motor(2)^2 - omega_motor(3)^2 + omega_motor(4)^2);
T = [Tx; Ty; Tz];
J = diag([Jx Jy Jz]);
omega = BRWxyz_g * [dphi dtheta dpsi]';
domega = J \(-cross(omega, J * omega) + T);
f = @(x, u) [
    omega(1)
    domega(1)
    omega(2)
    domega(2)
    domega(3)
];

dfdx = jacobian(f(x, u), x);
dfdu = jacobian(f(x, u), u);

% dfdu not function of u so u0 not needed
% u0 = [29000 29000 29000 29000]';

x0 = [0 0 0 0 0]';

dfdx_x0 = subs(dfdx, x, x0)
dfdu_u0 = dfdu % = subs(dfdu, u, u0)

% including params to solve for LQR gain
m = 0.027;                  % Mass
g = 9.81;                   % Gravitational acceleration
d = 0.046;                  % Distance from center of mass to rotor axis
lift = 1.9796E-9;           % Lift constant
drag =  2.5383E-11;         % Drag constant
b = lift;
k = drag;
Jx = 1.1463e-5;
Jy = 1.6993e-5;
Jz = 2.9944e-5;
dfdx_x0 = double(subs(dfdx_x0));
dfdu_u0 = double(subs(dfdu_u0));

% discretizing
sys = ss(dfdx_x0, dfdu_u0, zeros(5), 0);
h = 0.01;
sys_d = c2d(sys, h);
A_d = sys_d.a;
B_d = sys_d.b;

% weights for expensive controller -> better reference following
Q = diag([1000 1 1000 1 1]);
R = eye(4);

K = dlqr(A_d, B_d, Q, R)


function [dX,tau] = acrobot_noncollocated_linearization(t,X)
% dynamic equations of motion for acrobot, for use by ode45
% katiebyl 1/12/2010

% Define constants (geometry and mass properties):
% Below, parameters used in Spong94:
m1=1; m2=1; L1=1; L2=1; Lc1=.5; Lc2=.5; I1=.2; I2=1; g=9.8;
% Below, parameters used in Spong95:
% m1=1; m2=1; L1=1; L2=2; Lc1=.5; Lc2=1; I1=.083; I2=.33; g=9.8;

% Extract state variables from X:
q1 = X(1); q2 = X(2); dq1 = X(3); dq2 = X(4);

% Need to define torque somehow...
tau = 0;
TAU = [0; tau];

m11 = m1*Lc1^2 + m2*(L1^2 + Lc2^2 + 2*L1*Lc2*cos(q2)) + I1 + I2;
m22 = m2*Lc2^2 + I2;
m12 = m2*(Lc2^2 + L1*Lc2*cos(q2)) + I2;
m21 = m12;
M = [m11, m12; m21, m22];

h1 = -m2*L1*Lc2*sin(q2)*dq2^2 - 2*m2*L1*Lc2*sin(q2)*dq2*dq1;
h2 = m2*L1*Lc2*sin(q2)*dq1^2;
H = [h1;h2];

phi1 = (m1*Lc1+m2*L1)*g*cos(q1) + m2*Lc2*g*cos(q1+q2);
phi2 = m2*Lc2*g*cos(q1+q2);
PHI = [phi1; phi2];

% Now, determine torque, tau

alpha = 90*pi/180;
%q2des = (2*alpha/pi)*atan(dq1);  % collocated, eqn 53
q1des = 90*pi/180;                    % non-collocated, before eqn 56

% m=1; n=2. (underactuated)
T = [-(m11^-1)*(m12); 1];   % eqn 10
%M22bar = T'*M*T;   % collocated
M12T = m12'*(m12*m12')^-1; % non-collocated
M21tilda = m21 - m22*M12T*m11; % non-collocated
%h2bar = h2 - m21*(m11^-1)*h1;
%phi2bar = phi2 - m21*(m11^-1)*phi1;
h2tilda = h2 - m22*M12T*h1;   % non-collocated, after eqn 35
phi2tilda = phi2 - m22*M12T*phi1;  % non-collocated, after eqn 35

kf = 1;
kp=50*kf; kd=5*kf; % not selected with care!
%v2 = kp*(q2des - q2) - kd*dq2;   % collocated, eqn 55
v1 = kp*(q1des - q1) - kd*dq1;   % non-collocated, eqn 56
%tau = M22bar*v2 + h2bar + phi2bar;  % collocated, eqn 11
tau = M21tilda*v1 + h2tilda + phi2tilda;  % non-collocated, eqn 35


torque_limit = 1e20;  % [Nm] limit in torque magnitude
tau = sign(tau)*min(abs(tau),torque_limit);
TAU = [0; tau];

% M*d2Q + H + PHI = TAU
% d2Q = inv(M)*(TAU - H - PHI);

% Code BELOW wor
d2Q = (M^-1)*(TAU - H - PHI);  % Same EOM for collocated or non-coll.!
dX = [dq1; dq2; d2Q];



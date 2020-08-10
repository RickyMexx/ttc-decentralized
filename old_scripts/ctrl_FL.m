clear all; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Feedback Linearization control %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ----------- SETTINGS ----------- %

% Total time and sampling time (in ms)
T  = 2000;
dt = 1;

% Number of joints
n = 2; 

% Settings for 2R robot
m1 = 1;
d1 = 1;
l1 = 2;

m2 = 1;
d2 = 1;
l2 = 2;

g0 = 9.8;

% PD control
Kp = 0.01 * eye(n);
Kd = 0.1 * eye(n);

% Final conditions
qd    = [pi/2; pi/2];
dqd   = [0; 0];
ddqd  = [0; 0];

% Initial conditions
qs    = [0; 0];
qi    = qs;
dqi   = [0; 0];
ddqi  = [0; 0];
ui    = [0; 0];

% Angles wrapping to [-pi/2, pi/2]
qd = wrapToPi(qd);
qi = wrapToPi(qi);

ei    = qd-qs;
eprec = ei;

% Bounds
min_dq = [deg2rad(-400); deg2rad(-400)];
max_dq = [deg2rad(400); deg2rad(400)];

% -------------------------------- %

syms q [1 n] real
q = transpose(q);
% syms dq [1 n] real
% dq = transpose(dq);
% syms ddq [1 n] real
% ddq = transpose(ddq);

% 2R robot
m = [m1; m2];
a1 = m1 * d1^2 + m2 * d2^2 + m2 * l1^2;
a2 = m2 * l1 * d2;
a3 = m2 * d2^2;
a4 = g0*(m1*d1 + m2*l1);
a5 = g0 * m2 * d2;
a = [a1; a2; a3; a4; a5];

q1_plot = zeros(1,T);
q2_plot = zeros(1,T);
e1_plot = zeros(1,T);
e2_plot = zeros(1,T);
dq1_plot = zeros(1,T);
dq2_plot = zeros(1,T);
u1_plot = zeros(1,T);
u2_plot = zeros(1,T);

% Control scheme
for i=1:dt:T
    % Plots
    q1_plot(i) = qi(1);
    q2_plot(i) = qi(2);
    e1_plot(i) = ei(1);
    e2_plot(i) = ei(2);
    dq1_plot(i) = dqi(1);
    dq2_plot(i) = dqi(2);
    u1_plot(i) = ui(1);
    u2_plot(i) = ui(2);
    
    % Current error
    ei = double(qd - qi);
    
    % Global asymptotic stabilization -> a term at i-th iteration
    ai = ddqd + Kd * (ei - eprec) / dt + Kp * ei;
    
    % Updating precedent error for next derivatives
    eprec = ei;
    
    % Working in nominal conditions -> otherwise M and C are estimated
    ui =  eval_M(a, qi) * ai + eval_C(a, qi, dqi);
    
    % Entering in ROBOT model, using real M and C
    ui = ui - eval_C(a, qi, dqi);
    ddqi = double(eval_M(a, qi) \ (ui));
    
    %ddqi = clamp(ddqi, -0.05, 0.05); % clamping acceleration
    dqi = dqi + integrate(ddqi, dt);
    %dqi = clamp(dqi, min_dq, max_dq); % clamping velocity
    qi = qi + integrate(dqi, dt);
end


% --------------- Plots --------------- %
rows = 4;
cols = 2;

plotsubplot(rows, cols, 1, rad2deg(e1_plot), 'ms', 'deg', 'Joint 1: error', []);
plotsubplot(rows, cols, 2, rad2deg(e2_plot), 'ms', 'deg', 'Joint 2: error', []);

plotsubplot(rows, cols, 3, rad2deg(q1_plot), 'ms', 'deg', 'Joint 1: position', rad2deg(qd(1)) * ones(1,T));
plotsubplot(rows, cols, 4, rad2deg(q2_plot), 'ms', 'deg', 'Joint 2: position', rad2deg(qd(2)) * ones(1,T));

plotsubplot(rows, cols, 5, rad2deg(dq1_plot)*1000, 'ms', 'deg/s', 'Joint 1: velocity', []);
plotsubplot(rows, cols, 6, rad2deg(dq2_plot)*1000, 'ms', 'deg/s', 'Joint 2: velocity', []);

plotsubplot(rows, cols, 7, u1_plot*1000, 'ms', 'Nm', 'Joint 1: control torque', []);
plotsubplot(rows, cols, 8, u2_plot*1000, 'ms', 'Nm', 'Joint 2: control torque', []);

% ------------------------------------- %

function M = eval_M(a, q)
M = double([a(1) + 2*a(2)*cos(q(2)) a(3)+a(2)*cos(q(2)); a(3)+a(2)*cos(q(2)) a(3)]);
end

function C = eval_C(a, q, dq)
C = double([-a(2)*sin(q(2))*(dq(2)^2 + 2*dq(1)*dq(2)); a(2)*sin(q(2))*dq(1)^2]);
end

function G = eval_G(a, q)
G = double([a(4)*cos(q(1)) + a(5)*cos(q(1)+q(2)); a(5)*cos(q(1)+q(2))]);
end

function int = integrate(x, dt)
int = x * dt;
end

function c = clamp(val, val_min, val_max)
c = min(max(val, val_min), val_max);
end

function [] = plotsubplot(r, c, n, data, xlab, ylab, stitle, constant)
    subplot(r, c, n);
    plot(data);
    if not(isempty(constant))
        hold on;
        plot(constant);
    end
    xlabel(xlab);
    ylabel(ylab);
    title(stitle);
    grid on;
end
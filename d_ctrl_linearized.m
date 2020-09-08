clear all;
% Decentralized Control
% 2R robot w/ gravity

% Simulation time (ms)
T = 3000;
% Simulation step (ms)
dt  = 1;
idt = 1; % inve["ms", "rad"]rse dt

%% 2R parameters
m = [10; 10];
l = [1; 1];
d = [0.5; 0.5];
g0 = 0.0098;
[a, m] = eval_2r_params(l, d, m, g0);

% Initial conditions
qi  = [pi; 0]; % Initial joint state
dqi = [0; 0]; % Initial joint velocity
ddqi= [0; 0]; % Initial joint acceleration
ui  = [0; 0]; % Initial torque input
% Final conditions
qd  = [0; pi/2]; % Final joint state
dqd = [0; 0]; % Final joint velo city
ddqd= [0; 0]; % Final joint acceleration

% Robot variables
q  = qi;   % Joint state (variable)
dq = dqi;  % Joint velocity
ddq= ddqi; % Joint Acceleration
u  = ui;   % Torque input

% Joint bounds
min_dq = [-0.01, -0.01];
max_dq = [0.01, 0.01];

min_u = [-10, -10];
max_u = [10, 10];

min_q = [deg2rad(-360); deg2rad(-180)];
max_q = [deg2rad(+360); deg2rad(+180)];

%% Decentralized Joint Parameters
J = diag([1., 1.]); % Inertia matrix
D = diag([0.5, 0.5]); % Viscous Friction matrix
n = [0.5; 0.5];
N = diag(n); % Reduction ratio
Ni = N \ eye(size(N, 1));

Dr = Ni * D * Ni;

%% PD-Controller Parameters
kp = 0.3 * eye(2);
kd = 2  * eye(2);
err= qd - q;
err_prec = err;

num_steps = floor(T / dt);
data_mat(1:12,num_steps) = 0.0;

%% Simulation phase
for i = 1:dt:T
    % Update desired trajectory
    qd  = [sin(i/600); cos(i/300)];
    
    %% Update Plots
    data_mat = data_store(data_mat, i, q, dq, ddq, err, u, qd);
    % Compute robot's model in real time
    [Mbar, dM] = eval_2r_M_decomp(a, q);
    C = eval_2r_C(a, q, dq);
    G = eval_2r_G(a, q);
    % Compute perturbance and decentralized model
    d = Ni * dM * Ni * ddq + Ni * C * Ni * dq + Ni * G;
    Mbr = Ni * Mbar * Ni;
    
    % Decentralized Controller
    err = double(qd - q);
    derr = (err - err_prec) * idt;
    % Command unit with PD
    u = kd * derr + kp * err;
    %u(1) = max(min(max_u(1), u(1)), min_u(1));
    %u(2) = max(min(max_u(2), u(2)), min_u(2));
    
    % State equations
    x = [q; dq];
    dx = [x(3:4); -inv(Mbr) * (Dr * x(3:4))] + [zeros(2,2); inv(Mbr)] * (Ni * u);
    
    %% Update acceleration values and integrate    
    ddq = dx(3:4);
    dq  = dq + integrate(ddq, dt);
    %Constrain velocities
    %dq(1) = max(min(max_dq(1), dq(1)), min_dq(1));
    %dq(2) = max(min(max_dq(2), dq(2)), min_dq(2));
    q   = q  + integrate(dq, dt);    
    % Constrain joints
    q(1) = max(min(max_q(1), q(1)), min_q(1));
    q(2) = max(min(max_q(2), q(2)), min_q(2));
    % Update error
    err_prec = err;
end

%% Display results
%data_plot(data_mat, [["ms", "rad"]; ["ms", "rad"]; ["ms", "rad/ms"]; ["ms", "rad/ms"]], 2, 2, ["q1"; "q2"; "dq1"; "dq2"]);
labels = [["ms", "rad"]; ["ms", "rad"]; ["ms", "rad"]; ["ms", "rad"]; ["ms", "rad/ms"]; ["ms", "rad/ms"]; ["ms", "rad/ms^2"]; ["ms", "rad/ms^2"]; ["ms", "Nm"]; ["ms", "Nm"]];
titles = ["e1"; "e2"; "q1"; "q2"; "dq1"; "dq2"; "ddq1"; "ddq2"; "u1"; "u2"];
data_plot(data_mat, labels, 5, 2, titles);
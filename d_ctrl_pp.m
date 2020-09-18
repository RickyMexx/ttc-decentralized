clear all;
% Decentralized Control
% 2R robot w/ gravity

% Simulation time (ms)
T = 3000;
% Simulation step (ms)
dt  = 1;
idt = 1; % inve["ms", "rad"]rse dt

%% 2R parameters
m = [20; 20];
l = [1; 1];
d = [0.5; 0.5];
g0 = 0.0098;
[a, m] = eval_2r_params(l, d, m, g0);

% Initial conditions
qi  = [pi; pi]; % Initial joint state
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
um = ui;   % Motor torque

% Joint bounds
min_dq = [-0.01, -0.01];
max_dq = [0.01, 0.01];

min_u = [-10, -10];
max_u = [10, 10];

min_q = [deg2rad(-360); deg2rad(-180)];
max_q = [deg2rad(+360); deg2rad(+180)];

%% Decentralized Joint Parameters
J = diag([1., 1.]); % Inertia matrix
D = diag([0.9, 0.5]); % Viscous Friction matrix
N = diag([100; 100]); % Reduction ratio
Ni = N \ eye(size(N, 1));
Dr = Ni * D * Ni;


num_steps = floor(T / dt);
data_mat(1:12,num_steps) = 0.0;
err = qd - q;

% Reference gain matrix
Kr = diag([1 0.1]);

% Eigenvalues
P = [-1 -1.2 -1.3 -1.4];

% State equations
qm = N * q;
dqm = N * dq;
ddqm = N * ddq;
x = [qm; dqm];

%% Simulation phase
for i = 1:dt:T
    % Update desired trajectory
    qd  = [sin(i/600); cos(i/300)];
    u = Ni * um;
    % Update Plots
    data_mat = data_store(data_mat, i, q, dq, ddq, err, u, qd);
    
    % Compute robot's model in real time
    [Mbar, dM] = eval_2r_M_decomp(a, q);
    C = eval_2r_C(a, q, dq);
    G = eval_2r_G(a, q);
    % Compute perturbance and decentralized model
    d = Ni * dM * Ni * ddqm + Ni * C * Ni * dqm + Ni * G;
    Mbr = Ni * Mbar * Ni;
    
    x = [qm; dqm];
    A = [zeros(2) eye(2); zeros(2) -inv(Mbr) * Dr];
    B = [zeros(2); inv(Mbr)];
    
    dx = A * x + B * um;
    x = x + integrate(dx, dt);
    
    K = place(A, B, P);
    um = Kr * qd - K * x;
    
    % Motors update
    qm = x(1:2);
    dqm = x(3:4);
    ddqm = dx(3:4);
    
    % Joints update
    ddq = Ni * ddqm;
    dq = Ni * dqm;
    q = Ni * qm;
    
    % Updating error
    err = double(qd - q);
end

%% Display results
%data_plot(data_mat, [["ms", "rad"]; ["ms", "rad"]; ["ms", "rad/ms"]; ["ms", "rad/ms"]], 2, 2, ["q1"; "q2"; "dq1"; "dq2"]);
labels = [["ms", "rad"]; ["ms", "rad"]; ["ms", "rad"]; ["ms", "rad"]; ["ms", "rad/ms"]; ["ms", "rad/ms"]; ["ms", "rad/ms^2"]; ["ms", "rad/ms^2"]; ["ms", "Nm"]; ["ms", "Nm"]];
titles = ["e1"; "e2"; "q1"; "q2"; "dq1"; "dq2"; "ddq1"; "ddq2"; "u1"; "u2"];
data_plot(data_mat, labels, 5, 2, titles);
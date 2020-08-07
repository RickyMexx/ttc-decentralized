clear all;
% Decentralized Control
% 2R robot w/ gravity

% Simulation time (ms)
T = 2000;
% Simulation step (ms)
dt  = 1;
idt = 1; % inve["ms", "rad"]rse dt

%% 2R parameters
m = [1; 1];
l = [2; 2];
d = [1; 1];
g0 = 9.8;
[a, m] = eval_2r_params(l, d, m, g0);

% Initial conditions
qi  = [0; 0]; % Initial joint state
dqi = [0; 0]; % Initial joint velocity
ddqi= [0; 0]; % Initial joint acceleration
ui  = [0; 0]; % Initial torque input
% Final conditions
qd  = [pi/2; pi/2]; % Final joint state
dqd = [0; 0]; % Final joint velocity
ddqd= [0; 0]; % Final joint acceleration

% Robot variables
q  = qi;   % Joint state (variable)
dq = dqi;  % Joint velocity
ddq= ddqi; % Joint Acceleration
u  = ui;   % Torque input

% Joint bounds
min_dq = [-10; -10];
max_dq = [10; 10];

%% Decentralized Joint Parameters
J = diag([1., 1.]); % Inertia matrix
D = diag([1., 1.]); % Viscous Friction matrix
N = diag([200., 200.]); % Reduction ratio
Ni = inv(N);

%% PD-Controller Parameters
kp = 0.001 * eye(2);
kd = 0.01  * eye(2);
err= qd - q;
err_prec = err;

num_steps = floor(T / dt);
data_mat(1:10,num_steps) = 0.0;

%% Simulation phase
for i = 1:dt:T
    %% Update Plots
    data_mat = data_store(data_mat, i, q, dq, ddq, err, u);
    % Compute robot's model in real time
    [Mbar, dM] = eval_2r_M_decomp(a, q);
    C = eval_2r_C(a, q, dq);
    G = eval_2r_G(a, q);
    % Compute perturbance and decentralized model
    d = Ni * dM * Ni * ddq + Ni * C * Ni * dq + Ni * G;
    Mbr = Ni * Mbar * Ni;
    
    % Controller term
    err = double(qd - q);
    ai = kd * (err - err_prec) * idt + kp * err;
    % Compute command unit
    u = (J + Mbar) * ai + D * dq + d;
    % Update acceleration values and integrate
    ddq = inv(J + Mbr) * (u - D * dq - d);
    dq  = dq + integrate(ddq, dt);
    %Constrain velocities
    dq(1) = max(min(max_dq(1), dq(1)), min_dq(1));
    dq(2) = max(min(max_dq(2), dq(2)), min_dq(2));
    q   = q  + integrate(dq, dt);    
    % Update error
    err_prec = err;
end
%% Display results
data_plot(data_mat, [["ms", "rad"]; ["ms", "rad"]; ["ms", "rad/ms"]; ["ms", "rad/ms"]], 2, 2, ["q1"; "q2"; "dq1"; "dq2"]);
clear all;
% Decentralized Control
% 2R robot w/ gravity

% Simulation time (ms)
T = 2000;
% Simulation step (ms)
dt  = 1;
idt = 1; % inve["ms", "rad"]rse dt

%% 2R parameters
m = [100; 20];
l = [1; 1];
d = [1; 1];
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

min_q = [deg2rad(-360); deg2rad(-180)];
max_q = [deg2rad(+360); deg2rad(+180)];

%% Decentralized Joint Parameters
J = diag([1., 1.]); % Inertia matrix
D = diag([0.2, 0.2]); % Viscous Friction matrix
N = diag([1000., 400.]); % Reduction ratio
Ni = inv(N);

%% PD-Controller Parameters
kp = 50 * eye(2);
kd = 80  * eye(2);
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
    %% Update acceleration values and integrate
    %ddq = inv(J + Mbr) * (u - D * dq - d);
    % Update acceleration values wrt Dynamic Robot's model
    M = eval_2r_M(a, q);
    ddq = inv(M) * (Ni * u - C * dq - G);
    %%
    dq  = dq + integrate(ddq, dt);
    %Constrain velocities
    dq(1) = max(min(max_dq(1), dq(1)), min_dq(1));
    dq(2) = max(min(max_dq(2), dq(2)), min_dq(2));
    q   = q  + integrate(dq, dt);    
    % Constrain joints
    q(1) = max(min(max_q(1), q(1)), min_q(1));
    q(2) = max(min(max_q(2), q(2)), min_q(2));
    % Update error
    err_prec = err;
end
%% Display results
data_plot(data_mat, [["ms", "rad"]; ["ms", "rad"]; ["ms", "rad/ms"]; ["ms", "rad/ms"]], 2, 2, ["q1"; "q2"; "dq1"; "dq2"]);
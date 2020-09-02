clear all;
% Decentralized Control w/ Sliding Mode
% 2R robot w/ gravity

% Simulation time (ms)
T = 10000;
% Simulation step (ms)
dt  = 1;
idt = 1; % inve["ms", "rad"]rse dt

%% 2R Robot Parameters
m = [20; 20];
l = [1; 1];
d = [1; 1];
g0 = 0.0098;
[a, m] = eval_2r_params(l, d, m, g0);

% Initial conditions
qi  = [pi/2; pi/2]; % Initial joint state
dqi = [0; 0]; % Initial joint velocity
ddqi= [0; 0]; % Initial joint acceleration
ui  = [0; 0]; % Initial torque input
% Final conditions
%qd  = [sin(dt); cos(dt)]; % Final joint state
qd = [0; 0];
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
Ni = N \ eye(size(N, 1));

%% PD-Controller Parameters
kp = 25 * eye(2);
kd = 35  * eye(2);
err= qd - q;
err_prec = err;

%% Sliding Mode Parameters
gs = diag([30 30]); % Weight of error for the sliding surface
phi = [4 4]; % Boundary layers
ks = diag([10 2]); % Gain of the robust term

%% Data plot
num_steps = floor(T / dt);
data_mat(1:12,num_steps) = 0.0;

%% Simulation phase
for i = 1:dt:T
    % Update desired trajectory
    qd  = [sin(i/600); cos(i/300)]; % Test traj 1 
    %qd = [(i/6000)^3 + sin(i/1000); (i/6000)^2 + sin(i/1000)]; % Test traj 2 
    
    % Update Plots
    data_mat = data_store(data_mat, i, q, dq, ddq, err, u, qd);
    
    % Compute robot's model in real time
    [Mbar, dM] = eval_2r_M_decomp(a, q);
    C = eval_2r_C(a, q, dq);
    G = eval_2r_G(a, q);
    % Compute perturbance
    d = Ni * dM * Ni * ddq + Ni * C * Ni * dq + Ni * G;
    Mbr = Ni * Mbar * Ni;
    
    % Decentralized Controller
    err = double(qd - q);
    derr = (err - err_prec) * idt;
    s = gs * err + derr;
    sat(1,1) = eval_sat(s(1), phi(1));
    sat(2,1) = eval_sat(s(2), phi(2));
    % PD + Sliding Mode
    ai = kd * derr + kp * err + ks * sat;
    % Compute command unit
    u = (J + Mbar) * ai + D * dq + d;

    %% Update acceleration values and integrate
    % Update acceleration values wrt Dynamic Robot's model
    M = eval_2r_M(a, q);
    ddq = M \ (Ni * u - C * dq - G);
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
labels = [["ms", "rad"]; ["ms", "rad"]; ["ms", "rad"]; ["ms", "rad"]; ["ms", "rad/ms"]; ["ms", "rad/ms"]; ["ms", "rad/ms^2"]; ["ms", "rad/ms^2"]; ["ms", "Nm"]; ["ms", "Nm"]];
titles = ["e1"; "e2"; "q1"; "q2"; "dq1"; "dq2"; "ddq1"; "ddq2"; "u1"; "u2"];
data_plot(data_mat, labels, 5, 2, titles);

% Mean errors
%mean(data_mat(7,:))
%mean(data_mat(8,:))
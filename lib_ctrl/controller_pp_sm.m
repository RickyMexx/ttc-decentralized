function u = controller_pp_sm(q, dq, ddq, a, D, N, qd, err, derr)
%CONTROLLER_PP Pole Placement controller
%estimate
%[q,dq,ddq] : Robot State variables 
%       a   : Robot Dynamic coefficients
%       D   : Viscous Friction matrix
%       N   : gear-ratio matrix
%      qd   : Desired joint position
[Mbar, dM] = eval_2r_M_decomp(a, q);
Mbr = N\Mbar/N;
Dbr = N\D/N;

C = eval_2r_C(a, q, dq);
G = eval_2r_G(a, q);
Ni = N \ eye(2);
dqm = N * dq;
ddqm = N * ddq;
%d = Ni * dM * Ni * ddqm + Ni * C * Ni * dqm + Ni * G;
d = 0;

A = [zeros(2), eye(2); zeros(2), - inv(Mbr) * Dbr];
B = [zeros(2); inv(Mbr)];

% Choosing poles and placing them



x = [N\err; N\derr];

%% Sliding Mode
%% Tracking NOMINAL
%gs = diag([1 1]); % Weight of error for the sliding surface
%phi = [1 1]; % Boundary layers
%Ks = diag([1.6 0.16]); % Gain of the robust term
%P = [-3.3, -3.5, -5.6, -4.4];
%Kr = diag([3.35, 0.335]);
%K = place(A, B, P);
%% Tracking REAL
gs = diag([1 1]); % Weight of error for the sliding surface
phi = [1 1]; % Boundary layers
Ks = diag([0.8 0.25]); % Gain of the robust term
P = [-4.6, -4.3, -4.5, -6.4];
Kr = diag([9, 0.25]);
K = place(A, B, P);

% Sliding surface + saturation
s = gs * err + derr;
sat(1,1) = eval_2r_sat(s(1), phi(1));
sat(2,1) = eval_2r_sat(s(2), phi(2));

%% Final command
um = -K * x + Ks * sat + Kr * qd + d;
u = N * um;
end


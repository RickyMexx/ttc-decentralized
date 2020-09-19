function u = controller_pp_sm(q, dq, ddq, a, D, N, qd, err, derr)
%CONTROLLER_PP Pole Placement controller
%estimate
%[q,dq,ddq] : Robot State variables 
%       a   : Robot Dynamic coefficients
%       D   : Viscous Friction matrix
%       N   : gear-ratio matrix
%      qd   : Desired joint position
[Mbar, ~] = eval_2r_M_decomp(a, q);
Mbr = N\Mbar/N;
Dbr = N\D/N;

A = [zeros(2), eye(2); zeros(2), - inv(Mbr) * Dbr];
B = [zeros(2); inv(Mbr)];

% Choosing poles and placing them
P = [-1.5, -1.1, -1.2, -1.5];
K = place(A, B, P);
Kr = diag([5.5, 0.8]);

x = [N\q; N\dq];

%% Sliding Mode
gs = diag([2 2]); % Weight of error for the sliding surface
phi = [1 1]; % Boundary layers
Ks = diag([10.2 0.3]); % Gain of the robust term

% Sliding surface + saturation
s = gs * err + derr;
sat(1,1) = eval_2r_sat(s(1), phi(1));
sat(2,1) = eval_2r_sat(s(2), phi(2));

%% Final command
um = -K * x + Ks * sat + Kr * qd;
u = N * um;
end


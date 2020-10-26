function u = controller_lqr(q, dq, ddq, a, D, N, Q, R, qd, dqd)
%CONTROLLER_PP Pole Placement controller
%estimate
%[q,dq,ddq] : Robot State variables 
%       a   : Robot Dynamic coefficients
%       D   : Viscous Friction matrix
%       N   : gear-ratio matrix
%      qd   : Desired joint position
%      erj  : Error Rejection flag
[Mbar, dM] = eval_2r_M_decomp(a, q);
Mbr = N\Mbar/N;
Dbr = N\D/N;

% State Space matrices
A = [zeros(2), eye(2); zeros(2), - inv(Mbr) * Dbr];
B = [zeros(2); inv(Mbr)];

[K, ~, ~] = lqr(A, B, Q, R);
x = [N\(q - qd); N\(dq - dqd)];

um = -K * x;
%um = Kp * (qd-q) - K * x + d;
%um = Kr * qd - K * x + d;
%um = K * x;
u = N * um;
end
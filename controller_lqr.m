function u = controller_lqr(q, dq, ddq, a, D, N, qd, erj)
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

Ni = N \ eye(2);
dqm = N * dq;
ddqm = N * ddq;

if erj == true
    C = eval_2r_C(a, q, dq);
    G = eval_2r_G(a, q);    
    d = Ni * dM * Ni * ddqm + Ni * C * Ni * dqm + Ni * G;
else
    d = 0;
end

% State Space matrices
A = [zeros(2), eye(2); zeros(2), - inv(Mbr) * Dbr];
B = [zeros(2); inv(Mbr)];

Q = [1, 0, 0, 0;  % dq1
     0, 1, 0, 0;  % dq2
     0, 0, 1, 0;  % ddq1
     0, 0, 0, 1]; % ddq2
R = [2, 0;  % u1
     0, 2]; % u2

[K, ~, ~] = lqr(A, B, Q, R);
Kr = diag([0.1, 5]);

x = [N\q; N\dq];
%um = Kr * qd - K * x + d;
um = K * x;
u = N * um;
end
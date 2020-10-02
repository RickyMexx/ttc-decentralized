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
Cs = eye(4);

Q = [50, 0, 0, 0;  % q1
     0, 50, 0, 0;  % q2
     0, 0, 50, 0;  % dq1
     0, 0, 0, 50]; % dq2
R = [0.1, 0;  % u1
     0, 0.1]; % u2

[K, ~, ~] = lqr(A, B, Q, R);
Kr = diag([2.5, 2.5]);

x = [N\q; N\dq];

%um = Kr * qd - K * x + d;
um = -K * x;
u = N * um;
end
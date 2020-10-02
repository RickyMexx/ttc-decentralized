function u = controller_pp(q, dq, ddq, a, D, N, qd)
%CONTROLLER_PP Pole Placement controller
%estimate
%[q,dq,ddq] : Robot State variables 
%       a   : Robot Dynamic coefficients
%       D   : Viscous Friction matrix
%       N   : gear-ratio matrix
%      qd   : Desired joint position
%[Mbar, ~] = eval_2r_M_decomp(a, q);
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


P = [-1.8, -1.9, -1.1, -1.4];

K = place(A, B, P);
Kr = diag([5.2, 2.25]);
% Automatic Kr computation (Buggy AF!)
%Cs = eye(4);
%Acl = A - B*K;
%syscl = ss(Acl, B, Cs, [zeros(2); zeros(2)]);
%Kdc = dcgain(syscl);
%Kr = diag([1 / Kdc(1, 1), 1 / Kdc(2, 2)]);
%disp(Kr);


x = [N\q; N\dq];

um = Kr * qd - K * x + d;
%disp(-K * x);
%um = - K * x;
u = N * um;
end


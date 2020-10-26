function u = controller_pp_er(q, dq, ddq, a, D, N, P, Kr, qd, dqd)
%CONTROLLER_PP Pole Placement controller
%estimate
%[q,dq,ddq] : Robot State variables 
%       a   : Robot Dynamic coefficients
%       D   : Viscous Friction matrix
%       N   : gear-ratio matrix
%       P   : 
%[Mbar, ~] = eval_2r_M_decomp(a, q);
[Mbar, dM] = eval_2r_M_decomp(a, q);
Mbr = N\Mbar/N;
Dbr = N\D/N;

C = eval_2r_C(a, q, dq);
G = eval_2r_G(a, q);
Ni = N \ eye(2);
%dqm = N * dq;
%ddqm = N * ddq;
dqm = dq;
ddqm = ddq;
d = Ni * dM * Ni * ddqm + Ni * C * Ni * dqm + Ni * G;

A = [zeros(2), eye(2); zeros(2), - inv(Mbr) * Dbr];
B = [zeros(2); inv(Mbr)];

K = place(A, B, P);

x = [N\(q - qd); N\(dq - dqd)];
%x = [N\(q - qd); 0; 0];
um = K * x + d;
u = N * um;
end


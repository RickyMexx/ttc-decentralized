function u = controller_pp(q, dq, ddq, a, D, N, qd)
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

P = [-1.5, -1.1, -1.2, -1.5];

K = place(A, B, P);
Kr = diag([4.5, 0.6]);

x = [N\q; N\dq];

um = Kr * qd - K * x;
u = N * um;
end


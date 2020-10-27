function u = controller_pp(q, dq, ddq, a, D, N, P, qd, dqd)
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

A = [zeros(2), eye(2); zeros(2), - inv(Mbr) * Dbr];
B = [zeros(2); inv(Mbr)];

K = place(A, B, P);

x = [N\(q - qd); N\(dq - dqd)];
%x = [N\(q-qd); 0; 0];
um = - K * x;
u = N * um;
end


function u = controller_pp(q, dq, ddq, a, D, N, err, derr)
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
d = Ni * dM * Ni * ddqm + Ni * C * Ni * dqm + Ni * G;
%d = 0;

% Parameter for PP reg norm experiment
P = [-1.1, -1.2, -1.3, -1.4];
Kr = diag([1.0, 1.0]);
% Parameter for PP_EP reg norm experiment
%P = [-1.5 -1.23 -4.0 -4.2]; 
%Kr = diag([2.22, 0.05]); 
% Parameter for PP Tracking nom experiment
%P = [-3.3, -3.5, -5.6, -4.4];
%Kr = diag([3.35, 0.335]);
% Parameter for PP_EP Tracking nom experiment
%P = [-2.1 -1.5 -1.3 -1.4]; 
%Kr = diag([0, 0]); 
% Parameter for PP reg real experiment
%P = [-1.6, -2.3, -1.5, -1.4];
%Kr = diag([0.64, 0.02]);
% Parameter for PP_EP reg real experiment
%P = [-1.1, -1.2, -5, -5.5];
%Kr = diag([8, 0.025]);
% Parameter for PP Tracking real experiment
%P = [-4.6, -4.3, -4.5, -6.4];
%Kr = diag([9, 0.25]);
% Parameter for PP_EP Tracking real experiment
%P = [-2.1 -1.5 -1.3 -1.4]; 
%Kr = diag([0, 0]); 

A = [zeros(2), eye(2); zeros(2), - inv(Mbr) * Dbr];
B = [zeros(2); inv(Mbr)];


%P = [-1.5, -1.6, -1.1, -1.4];

K = place(A, B, P);
%Kr = diag([5.15, 1.6]);
% Automatic Kr computation (Buggy AF!)
%Cs = eye(4);
%Acl = A - B*K;
%syscl = ss(Acl, B, Cs, [zeros(2); zeros(2)]);
%Kdc = dcgain(syscl);
%Kr = diag([1 / Kdc(1, 1), 1 / Kdc(2, 2)]);
%disp(Kr);


x = [N\q; N\dq];

um = Kr * err - K * x + d;
u = N * um;
end


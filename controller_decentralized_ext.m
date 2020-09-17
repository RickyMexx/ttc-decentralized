function u = controller_decentralized_ext(q, dq, ddq, a, D, N, kp, kd, err, err_prec)
%CONTROLLER_DECENTRALIZED Decentralized controller extended with non-linear
%estimate
%[q,dq,ddq] : Robot State variables 
%       a   : Robot Dynamic coefficients
%       D   : Viscous Friction matrix
%       N   : gear-ratio matrix
%       kp  : PD-Proportional term
%       kd  : PD-Derivative term
%       err : State variable error
%  err_prec : previous State var. error


% Important notes from Siciliano. Handbook of Robotics:
% [305, (8.2)] N * q = qm
% [306, (8.3)] inv(N) * t = tm

[Mbar, dM] = eval_2r_M_decomp(a, q);
C = eval_2r_C(a, q, dq);
G = eval_2r_G(a, q);
Ni = inv(N);

% Compute ddq_targ
ddqm_targ = (kp * err + kd * (err - err_prec));
% Configuration dependant contribution
d = Ni * dM * Ni * ddqm_targ + Ni * C * Ni * (N * dq) + Ni * G;
% Convert to motor state variables
u_m = Ni * Mbar * Ni * ddqm_targ + (Ni * D * Ni) * (N * dq) + d;
u = N * u_m;
end

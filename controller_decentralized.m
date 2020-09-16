function u = controller_decentralized(q, dq, ddq, a, D, N, kp, kd, err, err_prec)
%CONTROLLER_DECENTRALIZED Decentralized controller
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

[Mbar, ~] = eval_2r_M_decomp(a, q);
Ni = inv(N);
% Compute ddq_targ
ddqm_targ = N * (kp * err + kd * (err - err_prec));
% Convert to motor state variables
u_m = Ni * Mbar * Ni * ddqm_targ + (Ni * D * Ni) * (N * dq);
u = N * u_m;
end


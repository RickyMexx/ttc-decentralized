function u = controller_gravity(q, dq, ddq, a, D, N, kp, kd, err, err_prec)
%CONTROLLER_GRAVITY compensate both gravity and coulomb frictions
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

C = eval_2r_C(a, q, dq);
G = eval_2r_G(a, q);

u = C * dq + D * dq + G;

end

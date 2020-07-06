function C = eval_2r_C(a, q, dq)
%EVAL_2R_C Return the centrifugal/coreolis term for a 2R robot
%   a : dynamic coefficients
%   q : joint states
%  dq : joint velocities
% 
C = double([
    -a(2) * sin(q(2)) * (dq(2)^2 + 2*dq(1)*dq(2));
    a(2) * sin(q(2)) * dq(1)^2
]);
end


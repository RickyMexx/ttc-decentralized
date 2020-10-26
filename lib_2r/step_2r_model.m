function [q, dq, ddq] = step_2r_model(dt, q, dq, u, a, D, q_constraints, dq_constraints, ddq_constraints)
%STEP_2R_MODEL Update the cinematic variables q,dq,ddq by applying a step
%to the 2R robot's model.
%      dt : integration time
%       q : joint position
%      dq : joint velocity
%     ddq : joint acceleration
%       u : control term
%       a : dynamic coefficients
%       D : viscous friction matrix
%  q_constraints : joint position constraints 
%  dq_constraints : joint velocity constraints
%  ddq_constraints : joint acceleration constraints
%
% *_constraints are matrices formed in the following way:
% [ a, b; c, d ] where the first row represent [min, max] for joint 1
% and the second row represent the same for joint 2
M = eval_2r_M(a, q);
C = eval_2r_C(a, q, dq);
G = eval_2r_G(a, q);



ddq = M\(u - (C + D) * dq - G);
%ddq(1) = max(min(ddq_constraints(1, 2), ddq(1)), ddq_constraints(1, 1));
%ddq(2) = max(min(ddq_constraints(2, 2), ddq(2)), ddq_constraints(2, 1));
dq = dq + integrate(ddq, dt);
% Constrain Velocities
%dq(1) = max(min(dq_constraints(1, 2), dq(1)), dq_constraints(1, 1));
%dq(2) = max(min(dq_constraints(2, 2), dq(2)), dq_constraints(2, 1));
q = q + integrate(dq, dt);
q(1) = max(min(q_constraints(1, 2), q(1)), q_constraints(1, 1));
q(2) = max(min(q_constraints(2, 2), q(2)), q_constraints(2, 1));
end


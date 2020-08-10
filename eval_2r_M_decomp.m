function [M, dM] = eval_2r_M_decomp(a, q)
%EVAL_2R_M Return the inertia matrix M of a 2R robot
%  a : dynamic coefficents
%  q : joint states
M = double(diag([a(1), a(3)]));
dM = double([
    2 * a(2) * cos(q(2)), a(3) + a(2) * cos(q(2));
    a(3) + a(2) * cos(q(2)), 0]);
end


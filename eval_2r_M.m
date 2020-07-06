function M = eval_2r_M(a, q)
%EVAL_2R_M Return the inertia matrix M of a 2R robot
%  a : dynamic coefficents
%  q : joint states
M = double([
    a(1) + 2 * a(2) * cos(q(2)), a(3) + a(2) * cos(q(2));
    a(3) + a(2) * cos(q(2)), a(3)]);
end


function G = eval_2r_G(a, q)
%EVAL_2R_G Return the gravity term for a 2R robot
%   a : dynamic coefficients
%   q : joint states
%

G = double([
    a(4) * cos(q(1)) + a(5) * cos(q(1) + q(2));
    a(5) * cos(q(1) + q(2))
]);
end


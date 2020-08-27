function G = eval_2r_G(a, q)
%EVAL_2R_G Return the gravity term for a 2R robot
%   a : dynamic coefficients
%   q : joint states
%
g_subterm1 = double(a(4) * cos(q(1)));
g_subterm2 = double(a(5) * cos( q(1) + q(2)));
G = double([
    g_subterm1 + g_subterm2;
    g_subterm2;
]);
end


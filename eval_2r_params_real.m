function [a, m] = eval_2r_params_real(l, d, m, g, err)
%EVAL_2R_PARAMS Computes the dynamic coefficents for a 2R robot.
% Slightly alter the coefficients to simulate nominal-real differences.
%     l : length for the links
%     d : CoM offset on X-axis
%     m : link mass
%     g : gravity component
%   err : nominal-real factor

m = m + err;
l = l + err;
d = d + err;

a(1:5) = 0.0;
a(1) = m(1) * d(1)^2 + m(2) * d(2)^2 + m(2) * l(1)^2;
a(2) = m(2) * l(1) * d(2);
a(3) = m(2) * d(2)^2;
a(4) = g * (m(1) * l(1) + m(2) * d(1));
a(5) = g * m(2) * l(2);
a = a';
end
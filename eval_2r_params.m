function [a, m] = eval_2r_params(l, d, m, g)
%EVAL_2R_PARAMS Computes the dynamic coefficents for a 2R robot
%     l : length for the links
%     d : CoM offset on X-axis
%     m : link mass
%     g : gravity component

I1zz = (1 / 12) * m(1) * l(1) ^ 2;
I2zz = (1 / 12) * m(2) * l(2) ^ 2;
a(1:5) = 0.0;
a(1) = I1zz + I2zz + m(1) * d(1)^2 + m(2) * d(2)^2 + m(2) * l(1)^2;
a(2) = m(2) * l(1) * d(2);
a(3) = I2zz + m(2) * d(2)^2;
a(4) = g * (m(1) * l(1) + m(2) * d(1));
a(5) = g * m(2) * l(2);
a = a';
end


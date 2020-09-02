function S = eval_2r_sat(s, phi)
%EVAL_2R_SAT Return the value of sat function for sliding mode control of a 2R robot
%  s : sliding surface
%  phi : boundary layers
S = sign(s);
if abs(s) <= phi
    S = s / phi;
end
end


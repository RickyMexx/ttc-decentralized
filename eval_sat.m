function S = eval_sat(s, phi)
%EVAL_SAT Return the value of sat function for sliding mode control
%  s : sliding surface
%  phi : boundary layers
S = sign(s);
if abs(s) <= phi
    S = s / phi;
end
end


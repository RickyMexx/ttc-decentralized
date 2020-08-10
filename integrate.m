function [I] = integrate(x,dt)
%INTEGRATE Integrate the numeric variable x, given the sampling time dt
%   x : numeric number, vector, matrix to be integrated
%  dt : sampling time
    I = x * dt;
end


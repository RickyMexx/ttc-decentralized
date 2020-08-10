function X = clamp(x, x_min, x_max)
%CLAMP Clamp the numeric value x between x_min and x_max
X = min(max(x, x_min), x_max);
end


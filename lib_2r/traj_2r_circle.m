function qd = traj_2r_circle(pc, r, l, t, T)
%TRAJ_2R_CIRCLE Returns the desired joints to track a circle at instant t
%  pc : Center point of the circle (xc, yc)
%  r : radius of the circle
%  l : links lenght (l1, l2)
%  t : time instant to evaluate
%  T : total simulation time

if l(1)+l(2) < abs(pc(1)) + r || l(1)+l(2) < abs(pc(2)) + r
    error('The circle trajectory is too far from the manipulator! Change the radius or the center.')
end

qd = zeros(2, 1);
tr = t;
alpha = (2*pi * tr) / T;

% Current point in the circle
xt = pc(1) + r*cos(alpha);
yt = pc(2) + r*sin(alpha);

c2 = real((xt^2 + yt^2 -l(1)^2 -l(2)^2)/(2*l(1)*l(2)));
s2 = real(sqrt(1-c2^2)); % Taking just the positive solution

qd(2) = atan2(s2, c2);
qd(1) = atan2(yt,xt) - atan2(l(2)*s2, l(1)+l(2)*c2);
end
% vehicle kinematics as Jacobian
% 2DW1C vehicle kinematics, q: angle, b: wheel distance
function [m] = J(q, b)

m = [cos(q)/2, cos(q)/2; sin(q)/2, sin(q)/2; -1/b, 1/b];

end
function dx = AirShipModel30(t,x)
global Fgb Fa Ft Fi mass Disturb

dx = zeros(12,1);

x(1);   %   x position
x(2);   %   y position
x(3);   %   z position
x(4);   %   pitch angle
x(5);   %   roll angle
x(6);   %   yaw angle
x(7);   %   x velocity
x(8);   %   y velocity
x(9);   %   z velocity
x(10);  %   pitch velocity
x(11);  %   roll velocity
x(12);  %   yaw velocity

%{
E = eye(6);
M_extend = blkdiag(E,mass);
F_extend = [x(7);x(8);x(9);x(10);x(11);x(12);F];
S = inv(M_extend)*F_extend;
%}
S = mass\(Fgb+Ft+Fa+Fi+Disturb);
dx(1) = x(7);
dx(2) = x(8);
dx(3) = x(9);
dx(4) = x(10);
dx(5) = x(11);
dx(6) = x(12);
dx(7) = S(1);
dx(8) = S(2);
dx(9) = S(3);
dx(10) = S(4);
dx(11) = S(5);
dx(12) = S(6);




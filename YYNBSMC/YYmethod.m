function f = YYmethod(x,i,delta_t)
%YYMETHOD Summary of this function goes here
%   Detailed explanation goes here
global mass

currenttime = i*delta_t;
yc =  [0 0 0 0 0 0]'%tracking signal;
k1 = 0.1;
c2 = 0.1;
lambda2 = 0.1;
epsilon2 = 0.1;


e1 = x(1:6)-yc;
alpha1 = k1*e1;
de1 = x(7:12)-dyc;
e2 = de1 + alpha1;

s2 = c2*e1 + e2;


f = mass*(-c2*(e2-alpha1)+inv(mass)*C*x(7:12)+ inv(mass)*D*x(7:12)+ddyc-k1*de1)...
    -lambdax*s2 - epsilon2*sign(s2);

end


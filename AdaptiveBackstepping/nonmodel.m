function dx=nonmodel(t,x)
global Fgb Fa Ft Fi mass Qall F Disturb

dx=zeros(6,1);
F=Fgb+Ft+Fa+Fi+Disturb;
Qall=inv(mass)*F;
dx(1)=Qall(1);
dx(2)=Qall(2);
dx(3)=Qall(3);
dx(4)=Qall(4);
dx(5)=Qall(5);
dx(6)=Qall(6);
ax=Qall(1);
ay=Qall(2);
az=Qall(3);
ap=Qall(4);
aq=Qall(5);
ar=Qall(6);
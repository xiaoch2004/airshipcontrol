%* ---------------------------------------------------------*
%*           此程序为偏航角控制，前飞速度控制                 *         
%*                   不控制控制俯仰角                        *
%*                   无滤波环节，无风                        * 
%*                                                          * 
%*                                                          * 
%* ---------------------------------------------------------*
close all
clear
clc
global Fgb Fa Ft Fi mass Qall F current_Efficiency Disturb
vol=35;R=2.55;g=9.8;pho=1;
m=72;
Rp=2.81;zG=28*5/m;xG=0;yG=0;xB=0;yB=0;zB=0;
B=pho*g*vol;
G=B;
G=m*g;
%% 转动惯量
ix=409.4260 ;iy=409.4482 ;iz=34.59410;
ixy=0;iyz=0;ixz=0;
%% 质量矩阵
m26=0;m35=0;m53=0;m62=0;
m11=10.8147;m22=10.8147;m33= 38.9521;m44=19.3024;m55=19.3024;m66=0.0;
mass=[m+m11 0     0       0       m*zG       -m*yG;
      0     m+m22 0      -m*zG     0           m26+m*xG;
      0     0     m+m33   m*yG     m35-m*xG    0;
      0    -m*zG   m*yG   ix+m44   -ixy       -ixz;
      m*zG  0      m53-m*xG  -ixy    iy+m55   -iyz;
      -m*yG m62+m*xG   0     -ixz     -iyz      iz+m66];
  
 Disturb = [0 0 0 0 0 0]';

 ThrustFault = [0 0 0 0 0 0 0 0]';
 Ft_actuator = [0 0 0 0 0 0 0 0]';
%% 控制分配矩阵
 matrix=[sqrt(2)/2,sqrt(2)/2,-sqrt(2)/2,-sqrt(2)/2,0,0,0,0;
        -sqrt(2)/2,sqrt(2)/2,sqrt(2)/2,-sqrt(2)/2,0,0,0,0;
        0    ,    0,    0,    0,    1,    1,    1,    1;
        0,0,0,0,sqrt(2)*Rp/2,sqrt(2)*Rp/2,-sqrt(2)*Rp/2,-sqrt(2)*Rp/2;
        0,0,0,0,-sqrt(2)*Rp/2,sqrt(2)*Rp/2,sqrt(2)*Rp/2,-sqrt(2)*Rp/2;
        -Rp,    -Rp,    -Rp,   -Rp,    0,    0,    0,    0];
sref=vol^(2/3);   
lref=vol^(1/3);  
CCx1=-[0.1388    0.1270    0.1111    0.1051    0.1044    0.1074    0.1062    0.1099    0.1294    0.1644    0.0726    0.0403    0.0081   -0.0320   -0.0921   -0.1286    -0.1617...
   -0.2469   -0.2663   -0.3297   -0.3928   -0.4804   -0.4364   -0.2544   -0.2020   -0.1101   -0.0793   -0.0681   -0.0468   -0.0186    0.0045];
CCz1= [0   -0.0398   -0.1165   -0.1615   -0.3319   -0.3867   -0.4492   -0.5146   -0.5878   -0.6873   -0.7122   -0.7856   -0.8402   -0.8416   -0.8048   -0.8694   -1.0206...
     -1.0926   -1.1755   -1.2886   -1.2424   -1.4046   -1.0869   -0.8941   -0.8802   -0.8364   -0.8321   -0.8351   -0.8363   -0.8241   -0.8306];
CCmy1=[0    0.0582    0.1212    0.1846    0.2303    0.2883    0.3391    0.3836    0.4209    0.4435    0.4784    0.4905    0.5045    0.5319    0.5693    0.5640    0.5184...
      0.4866    0.4605    0.4165    0.4183    0.3521    0.3691    0.2875    0.2336    0.1835    0.1451    0.1086    0.0738    0.0350   -0.0008];
%初始条件
phi=0;theta=0;psi=0;alpha=0;beta=0;px=0;py=0;pz=0;h=-pz;u=0;v=0;w=0;p=0;q=0;r=0;
%phi=pi/18;theta=pi/12;psi=pi/6;alpha=0;beta=0;px=-2;py=1.5;pz=-2;h=-pz;u=0;v=0;w=0;p=0;q=0;r=0;
eu0=0; eeu0=0;ew0=0;eew0=0;etheta0=0;eetheta0=0;epsi0=0;eepsi0=0;
ev0=0; eev0=0;
xk=[0,0,0,0,0,0];du_c=0;dw_c=0;dv_c=0;dtheta_c=0;dpsi_c=0;
mu1=0;mu2=0;mu3=0;mu4=0;
Ft1=0;Ft2=0;Ft3=0;Ft4=0;%%初始合力为浮力m*g/4
Ftx=0;Fty=0;Ftz=0;Fmx=0;Fmy=0;Fmz=0;
ft0=[0,0,0,0];mu0=[0 0 0 0];
ft=ft0;mu=mu0;
v10=zeros(6,1);
dv1=zeros(6,1);
fid = fopen('data.txt','wt');
fid2 = fopen('F_NU.txt','wt');
p1=zeros(6,1);
p2=zeros(6,1);
x2r=zeros(6,1);
x2r0=zeros(6,1);
dx2r=zeros(6,1);
uu=zeros(6,1);
uu0=zeros(6,1);
duu=zeros(6,1);
sp=[0.01 0.01 0.01 0.01 0.01 0.01]';

delt=0.01;
for i=0:delt:40
    Tspan=[0,delt];
%   time(i)=(i-1)*delt;

 RIB=[cos(psi)*cos(theta) cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
      sin(psi)*cos(theta) sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
      -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)];% 机体坐标系（速度）到惯性坐标系的转换矩阵
 A=[1 sin(phi)*tan(theta) cos(phi)*tan(theta);
    0       cos(phi)            -sin(phi);
    0 sin(phi)*sec(theta) cos(phi)*sec(theta)];%机体坐标系（角速度）到惯性坐标系的转换矩阵
 RBV=[cos(alpha)*cos(beta)      cos(alpha)*sin(beta) -sin(alpha);
      -sin(beta)           cos(beta)              0;
      sin(alpha)*cos(beta) sin(alpha)*sin(beta) cos(alpha)];%气流系到机体系转换矩阵
   alphaC=[0:3:90];
   windtype = 2;
[Vwd] = windfield(h, windtype);
Vwdd=1/1*[Vwd(1),Vwd(2),0]';
Vwdd=1/1*[0,0,0]';
%Vwb=inv(RIB)*Vwdd;将风速转换到机体坐标系
%Vwv=inv(RBV)*Vwb;
%% 沿三个坐标轴风速大小
uw=0;%Vwb(1);
vw=2*sin(i*5*pi/180);%Vwb(2);
ww=0;%Vwb(3);
VV=sqrt((u-uw)^2+(v-vw)^2+(w-ww)^2);%this is the relative velocity to wind to deside the aerodynamics forces

if (u-uw)~=0
      alpha=atan(abs((w-ww)/(sqrt((u-uw)^2+(v-vw)^2))));
end
 
if (u-uw)==0
      alpha=pi/2.0;
end

if (u-uw)==0&&(w-ww)==0
      alpha=0.0;
end

 alphad=alpha*180.0/pi;%将弧度转化成度数
%  zG
%  G
%  B

  Cx=interp1(alphaC,CCx1,alphad); 
  Cz=interp1(alphaC,CCz1,alphad); 
  Cmy=interp1(alphaC,CCmy1,alphad);
 %Cx=-0.13
 %Cmy=0.0
 %Cz=0.0
 %Cx=0.0
 Q=1/2*pho*VV^2;
 
 Cx = Cx + 10*rand(1,1)^2;
 
if (w-ww)>0.0
   Faz=Q*Cz*sref;
end

if (w-ww)<0.0
   Faz=-Q*Cz*sref;
end

if (w-ww)==0.0
   Faz=0.0;
end
%---------------------------------------------

if (u-uw)>0
    Sangle=atan((v-vw)/(u-uw));
    Fa=Q*Cx*sref;Fax=Fa*cos(Sangle);Fay=Fa*sin(Sangle);
    Ma=Q*Cmy*sref*lref;Max=-Ma*sin(Sangle);May=Ma*cos(Sangle);Maz=0.0;
end

if (u-uw)==0&&(v-vw)>0
    Sangle=pi/2.0;
    Fa=Q*Cx*sref;Fax=Fa*cos(Sangle);Fay=Fa*sin(Sangle);
    Ma=Q*Cmy*sref*lref;Max=-Ma*sin(Sangle);May=Ma*cos(Sangle);Maz=0.0;
end

if (u-uw)==0&&(v-vw)<0
    Sangle=-pi/2.0;
    Fa=Q*Cx*sref;Fax=Fa*cos(Sangle);Fay=Fa*sin(Sangle);
    Ma=Q*Cmy*sref*lref;Max=-Ma*sin(Sangle);May=Ma*cos(Sangle);Maz=0.0;
end

if (u-uw)==0&&(v-vw)==0
    Sangle=0.0;
    Fa=0.0;Fax=Fa*cos(Sangle);Fay=Fa*sin(Sangle);
    Ma=0.0;Max=-Ma*sin(Sangle);May=Ma*cos(Sangle);Maz=0.0;
end

if (u-uw)<0
    Sangle=atan((v-vw)/(u-uw));
    Fa=Q*Cx*sref;Fax=-Fa*cos(Sangle);Fay=-Fa*sin(Sangle);
    Ma=Q*Cmy*sref*lref;Max=Ma*sin(Sangle);May=-Ma*cos(Sangle);Maz=0.0;
end
Fa=[Fax Fay Faz Max May Maz]';
%% 相对螺旋桨轴的力分量 螺旋桨产生垂直力和水平面内的力,螺旋桨产生垂直向下的力时,矢量转角为0,螺旋桨绕矢量转轴顺时针为正,
Ft1h=Ft1*sin(mu1);Ft1v=-Ft1*cos(mu1);Ft2h=Ft2*sin(mu2);Ft2v=-Ft2*cos(mu2);Ft3h=Ft3*sin(mu3);Ft3v=-Ft3*cos(mu3);Ft4h=Ft4*sin(mu4);Ft4v=-Ft4*cos(mu4);
%Ftx=sqrt(2)/2*(Ft1h+Ft2h-Ft3h-Ft4h);Fty=sqrt(2)/2*(-Ft1h+Ft2h+Ft3h-Ft4h);Ftz=(Ft1v+Ft2v+Ft3v+Ft4v);
%Fmx=sqrt(2)/2*(Ft1v+Ft2v-Ft3v-Ft4v)*Rp;Fmy=sqrt(2)/2*(-Ft1v+Ft2v+Ft3v-Ft4v)*Rp;Fmz=(-Ft1h-Ft2h-Ft3h-Ft4h)*Rp;
 Disturb = [30*rand(1,1) 30*rand(1,1) 0 0.5*cos(i) 0.5*sin(i) 0]';
 %Disturb = [0 0 0 0 0 0]';
 ThrustFault = [4 0 0 0 0.5*sin(i) 0 0 0]';
 %ThrustFault = [0 0 0 0 0 0 0 0]';
 current_Efficiency = diag([1 1 1 1 1 1 1 1]);
 if(i>=30)
    current_Efficiency = diag([0 1 1 1 0 1 1 1]);
 end

 mass(1,1) = 72+10.8142+10*sin(i);
 mass(2,2)= 72+10.8142+10*sin(i);
 %mass(3,3)= mass(2,2) + 5*sin(i);
 %mass(4,4)= mass(4,4) + 10*sin(i);
 %mass(5,5)= 19.3024 + 4*sin(i);
 %mass(6,6)= 0.0 + 2*sin(i);
 Ft = matrix*(current_Efficiency * [Ft1h, Ft2h, Ft3h, Ft4h, Ft1v, Ft2v, Ft3v, Ft4v]' + ThrustFault);

%% FGB
Xgb=-(G-B)*sin(theta);
Ygb=(G-B)*sin(phi)*cos(theta);
Zgb=(G-B)*cos(phi)*cos(theta);
Lgb=-(G*zG-B*zB)*sin(phi)*cos(theta);
Mgb=-(G*zG-B*zB)*sin(theta)-(G*xG-B*xB)*cos(phi)*cos(theta);%B或 G大不同效果因为作用点不同 B不产生力矩
Ngb=(G*xG-B*xB)*sin(phi)*cos(theta);
Fgb=[Xgb,Ygb,Zgb,Lgb,Mgb,Ngb]';
%% FI
f1=-(m+m33)*w*q+(m+m22)*v*r-m*zG*p*r+m*xG*(r^2+q^2)-m*yG*p*q-m35*q^2+m26*r^2;
f2=-(m+m11)*u*r+(m+m33)*w*p-m*zG*q*r+m*yG*(r^2+p^2)+(m35-m*xG)*p*q;
f3=-(m+m22)*v*p+(m+m11)*u*q-m*yG*q*r+m*zG*(p^2+q^2)-m*xG*p*r;
f4=(m55-m66-iz+iy)*q*r+ixz*q*p-ixy*p*r-iyz*(r^2-p^2)-m*yG*(p*v-q*u)+m*zG*(r*u-p*w)-m62*v*q+m53*w*r-m35*q*v;
f5=(m66-m44-ix+iz)*p*r-ixz*(p^2-r^2)+ixy*q*r-iyz*q*p-m*zG*(q*w-r*v)+m*xG*(p*v-q*u)+m35*u*q;
f6=(m44-m55-iy+ix)*q*p+(iyz-ixz)*q*r-ixy*(q^2-p^2)-m*xG*(r*u-p*w)+m*yG*(q*w-r*v)-m53*w*p;
Fi=[f1 f2 f3 f4 f5 f6]';
  [t,x]=ode45('nonmodel',Tspan,xk);
   xk=x(length(x),:);
    u=xk(1);
    v=xk(2);
    w=xk(3);
    p=xk(4);
    q=xk(5);
    r=xk(6);
    
    VP=RIB*[u,v,w]';
    VA=A*[p,q,r]';
    phi=phi+VA(1)*delt;
    theta=theta+VA(2)*delt;
    psi=psi+VA(3)*delt; 
    px=px+VP(1)*delt;
    py=py+VP(2)*delt;
    pz=pz+VP(3)*delt;
%%
%{
h1=(1-1*exp(-0.1*(i)^3))*1;%(1-1*exp(-i^3))*1
dh1=0.1*(3*(i)^2*exp(-(i)^3))*1;
h2=pi/6*(1-1*exp(-i));
dh2=pi/6*(1*exp(-i));
h3=(3-2*exp(-0.1*(i-15)^3))*1;
dh3=(0.1*2*3*(i-15)^2*exp(-0.1*(i-15)^3))*1;
%}
N=inv(mass);
h1 = 1;
dh1 = 0;
h2 = pi/6;
dh2 = 0;
h3 = 2;
dh3 = 0;
x1d=[10*(2*sigmf(i,[0.1,0])-1);10*(2*sigmf(i,[0.1,0])-1);0;0;0;(pi/2)*(2*sigmf(i,[0.1,0])-1)];%[1;1;1]
dx1d=[dh1;dh1;dh1;0;0;dh2];%[0;0;0];
%if i>20
%    x1d=[2+1*exp(-0.1*(i-30)^3);2+1*exp(-0.1*(i-30)^3);2+1*exp(-0.1*(i-30)^3);0;0;pi/6*(1-1*exp(-i^3))];%[1;1;1]
%    dx1d=[0.1*3*(i-30)^2*exp(-(i-30)^3);0.1*3*(i-30)^2*exp(-(i-30)^3);0.1*3*(i-30)^2*exp(-(i-30)^3);0;0;pi/6*(1*3*i^2*exp(-i^3))];%[0;0;0];
%end

% x1d=[1;1;1;0;0;1];
% dx1d=[0;0;0;0;0;0];
x1=[px;py;pz;phi;theta;psi];
x2=[u;v;w;p;q;r];
g1=[cos(psi)*cos(theta) cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi) 0 0 0;
      sin(psi)*cos(theta) sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi) 0 0 0;
      -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi) 0 0 0;
      0 0 0 1 sin(phi)*tan(theta) cos(phi)*tan(theta);
      0 0 0 0 cos(phi)            -sin(phi);
      0 0 0 0 sin(phi)*sec(theta) cos(phi)*sec(theta) ];% 机体坐标系（速度）到惯性坐标系的转换矩阵
g2=N;
ff2=N*(Fgb+Fi);
c=eye(6,6);
c(1,1)=Q*sref*cos(Sangle);
c(2,2)=Q*sref*sin(Sangle);
c(3,3)=Q*sref;
c(4,4)=Q*sref*lref*sin(Sangle);
c(5,5)=Q*sref*lref*cos(Sangle);
c(6,6)=0;

z1=x1-x1d;
v1=g1\(-1.0*z1+dx1d);
dv1=(v1-v10)/delt;
v10=v1;
z2=x2-v1;
Tao=eye(6,6);
dsp=Tao*c'*g2'*z2;
sp0=sp;%记住上一时刻值
sp=sp+dsp*delt;
uu=g2\(-ff2+dv1-g1'*z1-1.0*z2)-c*sp;
 
    %% 求矢量推力  
Ftx=uu(1);Fty=uu(2);Ftz=uu(3);Fmx=uu(4);Fmy=uu(5);Fmz=uu(6);



FHV=pinv(matrix)*[Ftx,Fty,Ftz,Fmx,Fmy,Fmz]';% 水平跟竖直面上的分力，动态逆
 fth=zeros(4,1);
 ftv=zeros(4,1);
   for kk=1:4
        fth(kk)=FHV(kk);
        ftv(kk)=FHV(kk+4);
        ft(kk)=sqrt(fth(kk)^2+ftv(kk)^2);
        mu(kk)=atan2(fth(kk),-ftv(kk));
        %mu(kk)=(mu(kk)/pi)*180.0;
%% constraint

% if ft(kk)>2.5*g
%     ft(kk)=2.5*g;
% end
% if ft(kk)<-2.5*g
%     ft(kk)=-2.5*g;
% end
%% 滤波环节
                  k1=1;k2=1;
         ft(kk)=k1*ft(kk)+(1-k1)*ft0(kk);
         mu(kk)=k2*mu(kk)+(1-k2)*mu0(kk);
   end
    ft0=ft;mu0=mu;
    Ft1=ft(1);Ft2=ft(2);Ft3=ft(3);Ft4=ft(4);
%     fFt1(i)=ft(1);fFt2(i)=ft(2);fFt3(i)=ft(3);fFt4(i)=ft(4);
    mu1=mu(1);mu2=mu(2);mu3=mu(3);mu4=mu(4);

Cx=sp(1);
Cy=sp(1);
Cz=sp(3);
Cmx=-sp(5);
Cmy=sp(5);
Cmz=sp(6);

fprintf(fid,'%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e\n',i,x1d(1),x1(1),x1d(2),x1(2),x1d(3),x1(3),x1d(6),x1(6),x1d(4),x1(4),x1d(5),x1(5),Cx,Cy,Cz,Cmx,Cmy,Cmz);%19

    fprintf(fid2,'%e %e %e %e %e %e %e %e %e %e %e %e %e %e %e\n',i,Ft1,Ft2,Ft3,Ft4,mu1,mu2,mu3,mu4,vw,u,v,w,Fmy,Fmz);%15

%     Ue(i)=VP(1);Ve=VP(2);We=VP(3);
%     fmu1(i)=mu(1);fmu2(i)=mu(2);fmu3(i)=mu(3);fmu4(i)=mu(4);
%     fu(i)=u;fv(i)=v;fw(i)=w;fp(i)=p;fq(i)=q;fr(i)=r;
%     ftheta(i)=theta;fpsi(i)=psi;fphi(i)=phi;
%     fpx(i)=px;fpy(i)=py;fpz(i)=pz;fphi(i)=phi;ftheta(i)=theta;fpsi(i)=psi;
%     fft1(i)=Ft(1);fft2(i)=Ft(2);fft3(i)=Ft(3);fft4(i)=Ft(4);fft5(i)=Ft(5);fft6(i)=Ft(6);
%     ffa1(i)=Fa(1);ffa2(i)=Fa(2);ffa3(i)=Fa(3);ffa4(i)=Fa(4);ffa5(i)=Fa(5);ffa6(i)=Fa(6);
%     fpsi_c(i)=psi_c;
%     fu_c(i)=u_c;
i

end
fclose(fid);
fclose(fid2);

% %figure(1)
% subplot(2,1,1)
% plot(pp(:,1),pp(:,2),'b--',pp(:,1),pp(:,3),'k');
% ylabel('x(m)')
% xlabel('time (s)')
% legend('x1d','x')
% %figure(2)
% subplot(2,1,2)
% plot(pp(:,1),pp(:,4),'b--',pp(:,1),pp(:,5),'k');
% ylabel('y(m)')
% xlabel('time (s)')
% legend('y1d','y')
% figure(3)
% %subplot(2,2,3)
% plot(pp(:,1),pp(:,6),'b--',pp(:,1),pp(:,7),'k');
% ylabel('z(m)')
% xlabel('time (s)')
% legend('z1d','z')
% figure(4)
% %subplot(2,2,4)
% plot(pp(:,1),pp(:,8),'b--',pp(:,1),pp(:,9),'k');
% ylabel('psi(rad)')
% xlabel('time (s)')
% legend('psid','psi')
% % figure(5)
% % plot(pp(:,1),pp(:,9));
% % ylabel('Ty')
% % xlabel('time (s)')
% % figure(6)
% % plot(pp(:,1),pp(:,10));
% % ylabel('u')
% % xlabel('time (s)')
% % 
% figure(7)
% plot(jj(:,1),jj(:,2));
% ylabel('Ft1')
% xlabel('time (s)')
% figure(8)
% plot(jj(:,1),jj(:,3));
% ylabel('Ft2')
% xlabel('time (s)')
% figure(9)
% plot(jj(:,1),jj(:,4));
% ylabel('Ft3')
% xlabel('time (s)')
% figure(10)
% plot(jj(:,1),jj(:,5));
% ylabel('Ft4')
% xlabel('time (s)')
% figure(11)
% plot(jj(:,1),jj(:,6));
% ylabel('mu1')
% xlabel('time (s)')
% figure(12)
% plot(jj(:,1),jj(:,7));
% ylabel('mu2')
% xlabel('time (s)')
% figure(13)
% plot(jj(:,1),jj(:,8));
% ylabel('mu3')
% xlabel('time (s)')
% figure(14)
% plot(jj(:,1),jj(:,9));
% ylabel('mu4')
% xlabel('time (s)')
% 
% figure (15)
% subplot(2,2,1)
% plot(pp(:,1),pp(:,2)-pp(:,3));
% ylabel('Error x[m]')
% xlabel('time (s)')
% subplot(2,2,2)
% plot(pp(:,1),pp(:,4)-pp(:,5));
% ylabel('Error y[m]')
% xlabel('time (s)')
% subplot(2,2,3)
% plot(pp(:,1),pp(:,6)-pp(:,7));
% ylabel('Error z[m]')
% xlabel('time (s)')
% subplot(2,2,4)
% plot(pp(:,1),pp(:,8)-pp(:,9));
% ylabel('Error psi[m]')
% xlabel('time (s)')
% 
% figure(16)
% plot(pp(:,1),pp(:,10));
% ylabel('v1')
% xlabel('time (s)')
% %{
% figure (1)
% subplot(2,2,1)
% plot(time,fpsi,':',time,fpsi_c,'-');
% ylabel('\psi 偏航角(rad)');
% xlabel('time');
% legend('\psi','\psi_c');
% subplot(2,2,2)
% plot(time,fu,':',time,fu_c,'-');
% ylabel('u 前飞速度(m/s)');
% xlabel('time');
% legend('u','u_c');
% subplot(2,2,3)
% plot(time,fFt1,'-',time,fFt2,':',time,fFt3,'-.',time,fFt4,'--');
% ylabel('Ft 螺旋桨推力大小(N)');
% xlabel('time');
% legend('Ft1','Ft2','Ft3','Ft4');
% subplot(2,2,4)
% plot(time,fmu1,'-',time,fmu2,':',time,fmu3,'-.',time,fmu4,'--');
% ylabel('\mu 螺旋桨矢量转角(rad)');
% xlabel('time');
% legend('\mu1','\mu2','\mu3','\mu4');
% 
% 
% figure(2)
% subplot(2,2,1)
% plot(time,ftheta);
% ylabel('\theta 俯仰角(rad)');
% xlabel('time(s)');
% subplot(2,2,2)
% plot(time,fphi);
% ylabel('\phi 滚转角(rad)');
% xlabel('time(s)');
% subplot(2,2,3)
% plot(time,fv);
% ylabel('v 侧向速度(m/s)');
% xlabel('time(s)');
% subplot(2,2,4)
% plot(time,fw);
% ylabel('w 纵向速度(m/s)');
% xlabel('time(s)');
% 
% %}
% 
% % figure(20)
% % plot(time,Ue,'b',time,Ve,'g',time,We,'r');
% % legend('ue','ve','we');
% % xlabel('time');
% % ylabel('惯性系下速度');
% 
% % figure (1)
% % subplot(2,3,1)
% % plot(time,fu);
% % ylabel('u 前飞速度');
% % xlabel('time');
% % subplot(2,3,2)
% % plot(time,fv);
% % ylabel('v 侧向速度');
% % xlabel('time');
% % subplot(2,3,3)
% % plot(time,fw);
% % ylabel('w 纵向速度');
% % xlabel('time');
% % subplot(2,3,4)
% % plot(time,fp);
% % ylabel('p 滚转力矩');
% % xlabel('time');
% % subplot(2,3,5)
% % plot(time,fq);
% % ylabel('q 俯仰力矩');
% % xlabel('time');
% % subplot(2,3,6)
% % plot(time,fr);
% % ylabel('r 偏航力矩');
% % xlabel('time');
% % 
% % % figure (2)
% % % subplot(2,3,1)
% % % plot(time,fft1);
% % % ylabel('ft1 x轴向力大小');
% % % xlabel('time');
% % % subplot(2,3,2) 
% % % plot(time,fft2);
% % % ylabel('ft2 y侧向力大小');
% % % xlabel('time');
% % % subplot(2,3,3)
% % % plot(time,fft3);
% % % ylabel('ft3 z法向力大小');
% % % xlabel('time');
% % % subplot(2,3,4)
% % % plot(time,fft4);
% % % ylabel('ft4 滚转力矩大小');
% % % xlabel('time');
% % % subplot(2,3,5)
% % % plot(time,fft5);
% % % ylabel('ft5 俯仰力矩大小');
% % % xlabel('time');
% % % subplot(2,3,6)
% % % plot(time,fft6);
% % % ylabel('ft6 偏航力矩大小');
% % % xlabel('time');
% % 
% % % figure (3)
% % % subplot(2,3,1)
% % % plot(time,ffa1);
% % % ylabel('fa1 x轴气动力大小');
% % % xlabel('time');
% % % subplot(2,3,2)
% % % plot(time,ffa2);
% % % ylabel('fa2 y轴气动力大小');
% % % xlabel('time');
% % % subplot(2,3,3)
% % % plot(time,ffa3);
% % % ylabel('fa3 z轴气动力大小');
% % % xlabel('time');
% % % subplot(2,3,4)
% % % plot(time,ffa4);
% % % ylabel('fa4 滚转气动力大小');
% % % xlabel('time');
% % % subplot(2,3,5)
% % % plot(time,ffa5);
% % % ylabel('fa5 俯仰气动力大小');
% % % xlabel('time');
% % % subplot(2,3,6)
% % % plot(time,ffa6);
% % % ylabel('fa6 偏航气动力大小');
% % % xlabel('time');
% % 
% % figure (4)
% % subplot(2,2,1)
% % plot(time,ftheta);
% % ylabel('\theta 俯仰角');
% % xlabel('time');
% % subplot(2,2,2)
% % plot(time,fphi);
% % ylabel('\phi 滚转角');
% % xlabel('time');
% % subplot(2,2,3)
% % plot(time,fpsi,'-',time,fpsi_c,':');
% % ylabel('\psi 偏航角');
% % xlabel('time');
% % subplot(2,2,4)
% % plot(fpx,fpy);
% % ylabel('y');
% % xlabel('x');
% % 
% % 
% % figure(20)
% % subplot(1,2,1)
% % plot(time,fFt1,'-',time,fFt2,'-.',time,fFt3,'--',time,fFt4,'.-')
% % ylabel('各螺旋桨推力(N)');
% % xlabel('time(s)');
% % grid on
% % subplot(1,2,2)
% % plot(time,fmu1,'-',time,fmu2,'-.',time,fmu3,'--',time,fmu4,'.-')
% % ylabel('各螺旋桨转角(rad)');
% % xlabel('time(s)');
% % grid on
% % % figure(5)
% % % subplot(1,3,1)
% % % plot(time,fpx);
% % % xlabel('time');
% % % ylabel('x轴方向位置');
% % % subplot(1,3,2)
% % % plot(time,fpy);
% % % xlabel('time');
% % % ylabel('y轴方向位置');
% % % subplot(1,3,3)
% % % plot(time,fpz);
% % % xlabel('time');
% % % ylabel('z轴方向位置');
% % 
% % figure (6)
% % subplot(2,2,1)
% % plot(time,fFt1);
% % ylabel('Ft1 1号螺旋桨推力');
% % xlabel('time');
% % subplot(2,2,2)
% % plot(time,fFt2);
% % ylabel('Ft2 2号螺旋桨推力');
% % xlabel('time');
% % subplot(2,2,3)
% % plot(time,fFt3);
% % ylabel('Ft3 3号螺旋桨推力');
% % xlabel('time');
% % subplot(2,2,4)
% % plot(time,fFt4);
% % ylabel('Ft4 4号螺旋桨推力');
% % xlabel('time');
% % % 
% % figure(7)
% % subplot(2,2,1)
% % plot(time,fmu1);
% % ylabel('\mu1 1号螺旋桨转角');
% % xlabel('time');
% % subplot(2,2,2)
% % plot(time,fmu2);
% % ylabel('\mu2 2号螺旋桨转角');
% % xlabel('time');
% % subplot(2,2,3)
% % plot(time,fmu3);
% % ylabel('\mu3 3号螺旋桨转角');
% % xlabel('time');
% % subplot(2,2,4)
% % plot(time,fmu4);
% % ylabel('\mu4 4号螺旋桨转角');
% % xlabel('time');
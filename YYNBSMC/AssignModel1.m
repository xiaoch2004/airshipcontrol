%% 3M airship characters
global mass
vol=70;m=72;R=3;g=9.8;rho=1.29;             %外形参数
Rp=3;                                    %矢量螺旋桨的安装距离
zG=28*5/m;xG=0;yG=0;xB=0;yB=0;zB=0;      %体心坐标
B=rho*g*vol;                             %浮力参数
G=B;                                     %重力参数
gammax=500;gammay=500;gammaz=200;

%% 气动力参数 （迎角为0到90度时，每隔3度，量测一个气动力系统，得到如下31个气动力系数）
sref=vol^(2/3);   
lref=vol^(1/3);  
CCx1=-[0.1388    0.1270    0.1111    0.1051    0.1044    0.1074    0.1062    0.1099    0.1294    0.1644    0.0726    0.0403    0.0081   -0.0320   -0.0921   -0.1286    -0.1617...
   -0.2469   -0.2663   -0.3297   -0.3928   -0.4804   -0.4364   -0.2544   -0.2020   -0.1101   -0.0793   -0.0681   -0.0468   -0.0186    0.0045];
CCz1= [0   -0.0398   -0.1165   -0.1615   -0.3319   -0.3867   -0.4492   -0.5146   -0.5878   -0.6873   -0.7122   -0.7856   -0.8402   -0.8416   -0.8048   -0.8694   -1.0206...
     -1.0926   -1.1755   -1.2886   -1.2424   -1.4046   -1.0869   -0.8941   -0.8802   -0.8364   -0.8321   -0.8351   -0.8363   -0.8241   -0.8306];
CCmy1=[0    0.0582    0.1212    0.1846    0.2303    0.2883    0.3391    0.3836    0.4209    0.4435    0.4784    0.4905    0.5045    0.5319    0.5693    0.5640    0.5184...
      0.4866    0.4605    0.4165    0.4183    0.3521    0.3691    0.2875    0.2336    0.1835    0.1451    0.1086    0.0738    0.0350   -0.0008];

% Inertia matrix
ix=409.4260 ;iy=409.4482 ;iz=34.59410;
ixy=0;iyz=0;ixz=0;

% mass matrix
m26=0;m35=0;m53=0;m62=0;
m11=10.8147;m22=10.8147;m33= 38.9521;m44=19.3024;m55=19.3024;m66=0.0;
mass=[m+m11       0           0          0         m*zG          -m*yG;
      0          m+m22        0        -m*zG        0           m26+m*xG;
      0           0          m+m33      m*yG      m35-m*xG         0;
      0          -m*zG       m*yG      ix+m44     -ixy           -ixz;
      m*zG        0         m53-m*xG    -ixy      iy+m55         -iyz;
     -m*yG     m62+m*xG       0         -ixz       -iyz         iz+m66];

angle=sqrt(2)/2;
CA = [angle angle -angle -angle 0 0 0 0;-angle angle angle -angle 0 0 0 0;0 0 0 0 1 1 1 1;
    0 0 0 0 angle*Rp angle*Rp -angle*Rp -angle*Rp; 0 0 0 0 -angle*Rp angle*Rp angle*Rp -angle*Rp;-Rp -Rp -Rp -Rp 0 0 0 0];


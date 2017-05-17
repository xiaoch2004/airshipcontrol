%% 30M airship characters
m=1007.3252;vol=9800;g=9.8;rho=1/14;Rp=16;% thrust position

zG=3.7;
xG=0;yG=0;xB=0;yB=0;zB=0;gammax=500;gammay=500;gammaz=100;
MUmax=pi; delta_tMUmax=pi/1;

 Fmax=200;
 delta_tFmax=10; %每秒内的变化量 
 ix=2.1855e+005 ;iy=2.1855e+005;iz=1.3953e+005;ixy=0;iyz=0;ixz=0;B=rho*g*vol;G=B;
 m26=0*rho;m35=0*rho;m53=0*rho;m62=0*rho;m11=3324.12*rho;m22=3324.12*rho;m33=8496*rho;m44=28176.8*rho;m55=28176.8*rho;m66=0.0;
 mass=[m+m11 0     0       0       m*zG       -m*yG;
       0     m+m22 0      -m*zG     0           m26+m*xG;
       0     0     m+m33   m*yG     m35-m*xG    0;
       0    -m*zG   m*yG   ix+m44   -ixy       -ixz;
       m*zG  0      m53-m*xG  -ixy    iy+m55   -iyz;
       -m*yG m62+m*xG   0     -ixz     -iyz      iz+m66];
 
sref=vol^(2/3);   
lref=vol^(1/3);

 CCx1=-[0.1388    0.1270    0.1111    0.1051    0.1044    0.1074    0.1062    0.1099    0.1294    0.1644    0.0726    0.0403    0.0081   -0.0320   -0.0921   -0.1286    -0.1617...
       -0.2469   -0.2663   -0.3297   -0.3928   -0.4804   -0.4364   -0.2544   -0.2020   -0.1101   -0.0793   -0.0681   -0.0468   -0.0186    0.0045];
 CCz1= [0   -0.0398   -0.1165   -0.1615   -0.3319   -0.3867   -0.4492   -0.5146   -0.5878   -0.6873   -0.7122   -0.7856   -0.8402   -0.8416   -0.8048   -0.8694   -1.0206...
       -1.0926   -1.1755   -1.2886   -1.2424   -1.4046   -1.0869   -0.8941   -0.8802   -0.8364   -0.8321   -0.8351   -0.8363   -0.8241   -0.8306];
 CCmy1=[0    0.0582    0.1212    0.1846    0.2303    0.2883    0.3391    0.3836    0.4209    0.4435    0.4784    0.4905    0.5045    0.5319    0.5693    0.5640    0.5184...
       0.4866    0.4605    0.4165    0.4183    0.3521    0.3691    0.2875    0.2336    0.1835    0.1451    0.1086    0.0738    0.0350   -0.0008];
 CC_Nr = -0.214;        %滚转阻尼系数，产生原因纯摩擦和气动力，实验得到
 CC_Lp = 2*CC_Nr;       %角速度阻尼，无实验数据，产生因素为摩擦力+气动力，估算为C_Nr的2倍


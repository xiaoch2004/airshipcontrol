clear all; close all; clc;

global Fgb Fa Ft Fi Disturb
   
   %% Choose Model
 AssignModel1;
 
%% Initializing States
 current_phi=pi/18;
 current_theta=pi/12;
 current_psi=-pi/6;
 current_px=5;
 current_py=1.5;
 current_pz=-2;
 current_u=1;
 current_v=1;
 current_w=0.5;
 current_p=0.3;
 current_q=0.2;
 current_r=0.1;
 currentState=[current_px current_py current_pz current_phi current_theta current_psi current_u current_v current_w current_p current_q current_r]'; 
 current_U_p=0;
 alpha = 0;
 beta = 0;
 current_Efficiency = eye(8);%diag([1 1 1 1 1 1 1 1]);
 Disturb = [0 0 0 0 0 0]';
 ThrustFault = [0 0 0 0 0 0 0 0]';
 Ft_actuator = [0 0 0 0 0 0 0 0]';
 
 mu1=0;mu2=0;mu3=0;mu4=0;   %螺旋桨角度
 Ft1=0;Ft2=0;Ft3=0;Ft4=0;   %螺旋桨推力
 
 % Time parameters
 delta_t = 0.01;
 STEP = 4000;
 timespan = 0:delta_t:STEP*delta_t;
 %x_des = GenerateDes(timespan);

 
 fid1 = fopen('States.txt','wt');
 %fid2 = fopen('SlidingSurfaces.txt','wt');
 fid3 = fopen('inputForces.txt','wt');
 %fid4 = fopen('ParaEstimation.txt','wt');
 fid5 = fopen('GroundStates.txt','wt');
 
 %{
 fprintf(fid1,'%8s %8s %8s %8s %8s %8s %8s %8s %8s %8s %8s %8s %8s\n','time','px','py','pz','phi','theta','psi','u','v','w','p','q','r');
 fprintf(fid2,'%8s %8s %8s %8s %8s %8s %8s\n','time','S1','S2','S3','S4','S5','S6');
 %}
 
 controlAlloc_const = sqrt(2)/2;
 D = [controlAlloc_const controlAlloc_const -controlAlloc_const -controlAlloc_const 0 0 0 0;
     -controlAlloc_const controlAlloc_const controlAlloc_const -controlAlloc_const 0 0 0 0;
     0 0 0 0 1 1 1 1;
     0 0 0 0 controlAlloc_const*Rp controlAlloc_const*Rp -controlAlloc_const*Rp -controlAlloc_const*Rp;
     0 0 0 0 -controlAlloc_const*Rp controlAlloc_const*Rp controlAlloc_const*Rp -controlAlloc_const*Rp;
     -Rp -Rp -Rp -Rp 0 0 0 0];
 D_norm = norm(D);
 
%% Tracking signal
 
%% Generate Wind
 %Vwd_List = normrnd(0,0.5,3,10);   % Gaussian Wind
 %Vwd_List = load('Vwd_List.txt');   % Read last record
 %save Vwd_List.txt Vwd_List;       % Save current wind
 %Vwd = Vwd_List(:,1);               % Init wind set
 %Vwd_List = GenerateWind(STEP,3,delta_t);
 Vwd = [0 0 0]';
%% The Big Loop
for i=1:1:STEP
    currenttime = i*delta_t;

    %current_x_des = GenerateDes(currenttime);
    current_x_des = [0 0 0 0 0 0]';
    
    outputCompleteRateStep = 100;
    if mod(i,STEP/outputCompleteRateStep)==0
        rate=i/STEP;
        fprintf('%0.2f%% complete!\n',rate*100);
    end
     %Vwd = Vwd_List(:,i);  % Wind List
	 Tspan=[(i-1)*delta_t,i*delta_t];
	 %time(i)=(i-1)*delta_t;
	 RIB=[cos(current_psi)*cos(current_theta) cos(current_psi)*sin(current_theta)*sin(current_phi)-sin(current_psi)*cos(current_phi) cos(current_psi)*sin(current_theta)*cos(current_phi)+sin(current_psi)*sin(current_phi);
		  sin(current_psi)*cos(current_theta) sin(current_psi)*sin(current_theta)*sin(current_phi)+cos(current_psi)*cos(current_phi) sin(current_psi)*sin(current_theta)*cos(current_phi)-cos(current_psi)*sin(current_phi);
		  -sin(current_theta) cos(current_theta)*sin(current_phi) cos(current_theta)*cos(current_phi)];         % Body（Linear Velocity）to inertial coordinate transfer matrix
	 A = [1 sin(current_phi)*tan(current_theta) cos(current_phi)*tan(current_theta);
		0       cos(current_phi)            -sin(current_phi);
		0 sin(current_phi)*sec(current_theta) cos(current_phi)*sec(current_theta)]; % Body（Angular Velocity）to inertial coordinate transfer matrix
	 RBV=[cos(alpha)*cos(beta)      cos(alpha)*sin(beta) -sin(alpha);
		  -sin(beta)           cos(beta)              0;
		  sin(alpha)*cos(beta) sin(alpha)*sin(beta) cos(alpha)];      % Air to Body coordinate transfer matrix
    
      J1 = RIB;
      J2 = A;
      J = blkdiag(J1,J2);
      M_eta = mass/J;
%% Fa    
	alphaC=0:3:90;
	Vwb=RIB\Vwd;                   % Wind transferred to body coordinate(It's inv(RIB)*Vwd)
	uw=Vwb(1);
	vw=Vwb(2);
	ww=Vwb(3);
	VV=sqrt((current_u-uw)^2+(current_v-vw)^2+(current_w-ww)^2);    % the aerodynamics forces dependents on relative velocity to wind
	if (current_u-uw)~=0
		  alpha=atan(abs((current_w-ww)/(sqrt((current_u-uw)^2+(current_v-vw)^2))));
	end
	if (current_u-uw)==0
		  alpha=pi/2.0;
	end
	if (current_u-uw)==0&&(current_w-ww)==0
		  alpha=0.0;
	end
	 alphad=alpha*180.0/pi;% rad to degree
	 Cx=interp1(alphaC,CCx1,alphad); 
	 Cz=interp1(alphaC,CCz1,alphad); 
	 Cmy=interp1(alphaC,CCmy1,alphad);
     
	 %Cx = Cx + rand(1,1);
	 
     Q=1/2*rho*VV^2;
	if (current_w-ww)>0.0
	   Faz=Q*Cz*sref;
	end
	if (current_w-ww)<0.0
	   Faz=-Q*Cz*sref;
	end
	if (current_w-ww)==0.0
	   Faz=0.0;
	end
	%---------------------------------------------
	if (current_u-uw)>0
		Sangle=atan((current_v-vw)/(current_u-uw));
		Fa=Q*Cx*sref;Fax=Fa*cos(Sangle);Fay=Fa*sin(Sangle);
	end
	if (current_u-uw)==0&&(current_v-vw)>0
		Sangle=pi/2.0;
		Fa=Q*Cx*sref;Fax=Fa*cos(Sangle);Fay=Fa*sin(Sangle);
	end
	if (current_u-uw)==0&&(current_v-vw)<0
		Sangle=-pi/2.0;
		Fa=Q*Cx*sref;Fax=Fa*cos(Sangle);Fay=Fa*sin(Sangle);
	end
	if (current_u-uw)==0&&(current_v-vw)==0
		Sangle=0.0;
		Fa=0.0;Fax=Fa*cos(Sangle);Fay=Fa*sin(Sangle);
	end
	if (current_u-uw)<0
		Sangle=atan((current_v-vw)/(current_u-uw));
		Fa=Q*Cx*sref;Fax=-Fa*cos(Sangle);Fay=-Fa*sin(Sangle);
	end
	Ma=Q*Cmy*sref*lref;
	Max=-Ma*sin(Sangle); May=Ma*cos(Sangle); Maz=0;
    Max=Max-gammax*current_p; May=May-gammay*current_q; Maz=-gammaz*current_r;
	%Max=Max-sign(current_p)*5000*current_p^2; May=May-sign(current_q)*5000*current_q^2; Maz=-sign(current_r)*5000*current_r^2;
    Fa=[Fax Fay Faz Max May Maz]';
    
	%% FGB
	Xgb=-(G-B)*sin(current_theta);
	Ygb=(G-B)*sin(current_phi)*cos(current_theta);
	Zgb=(G-B)*cos(current_phi)*cos(current_theta);
	Lgb=-(G*zG-B*zB)*sin(current_phi)*cos(current_theta);
	Mgb=-(G*zG-B*zB)*sin(current_theta)-(G*xG-B*xB)*cos(current_phi)*cos(current_theta);    % B或G大不同效果因为作用点不同 B不产生力矩
	Ngb=(G*xG-B*xB)*sin(current_phi)*cos(current_theta);
	Fgb=[Xgb Ygb Zgb Lgb Mgb Ngb]';
	%% FI
    
	f1=-(m+m33)*current_w*current_q+(m+m22)*current_v*current_r-m*zG*current_p*current_r+m*xG*(current_r^2+current_q^2)-m*yG*current_p*current_q-m35*current_q^2+m26*current_r^2;
	f2=-(m+m11)*current_u*current_r+(m+m33)*current_w*current_p-m*zG*current_q*current_r+m*yG*(current_r^2+current_p^2)+(m35-m*xG)*current_p*current_q;
	f3=-(m+m22)*current_v*current_p+(m+m11)*current_u*current_q-m*yG*current_q*current_r+m*zG*(current_p^2+current_q^2)-m*xG*current_p*current_r;
	f4=(m55-m66-iz+iy)*current_q*current_r+ixz*current_q*current_p-ixy*current_p*current_r-iyz*(current_r^2-current_p^2)-m*yG*(current_p*current_v-current_q*current_u)+m*zG*(current_r*current_u-current_p*current_w)-m62*current_v*current_q+m53*current_w*current_r-m35*current_q*current_v;
	f5=(m66-m44-ix+iz)*current_p*current_r-ixz*(current_p^2-current_r^2)+ixy*current_q*current_r-iyz*current_q*current_p-m*zG*(current_q*current_w-current_r*current_v)+m*xG*(current_p*current_v-current_q*current_u)+m35*current_u*current_q;
	f6=(m44-m55-iy+ix)*current_q*current_p+(iyz-ixz)*current_q*current_r-ixy*(current_q^2-current_p^2)-m*xG*(current_r*current_u-current_p*current_w)+m*yG*(current_q*current_w-current_r*current_v)-m53*current_w*current_p;
	Fi=[f1 f2 f3 f4 f5 f6]';
    
    %% Ft
    %if i==8000
    %ThrustFault = [0 0 0 0 0 0 0 0]';
    %end%Add Faults
    %current_Efficiency = diag([1 1 1 1 1 1 1 1]);
    %if(currenttime>=30)
        %current_Efficiency = diag([0 1 1 1 0 1 1 1]);
        %Cx = Cx + 10*rand(1,1)^2;
    %end
        %Disturb = [30*randn(1,1) 30*randn(1,1) 0 0.5*cos(currenttime) 0.5*sin(currenttime) 0]';
        %mass(1,1) = 72 + 10.8147 + 10*sin(currenttime);
        %mass(2,2) = 72 + 10.8147 + 10*sin(currenttime);
        %mass(5,5) = 409.4482 + 19.3024 + 50*cos(currenttime);
        %ThrustFault = [4 0 0 0 0.5*sin(currenttime) 0 0 0]';
    %current_Efficiency = diag([1 1 1 1 1 1 1 1]);
    %Disturb = [0 0 0 0 0 0]';

    %mass(3,3) = mass(3,3) + 5*sin(currenttime);
    %mass(4,4)= mass(4,4) + 10*sin(currenttime);
    %mass(5,5) = mass(5,5) + 10*sin(currenttime);
    %mass(6,6) = mass(6,6) + sin(currenttime);
    
    
    %Ft = D * (current_Efficiency * Ft_actuator + ThrustFault);
    Ft1h=Ft1*sin(mu1);Ft1v=-Ft1*cos(mu1);Ft2h=Ft2*sin(mu2);Ft2v=-Ft2*cos(mu2);Ft3h=Ft3*sin(mu3);Ft3v=-Ft3*cos(mu3);Ft4h=Ft4*sin(mu4);Ft4v=-Ft4*cos(mu4);
    Ftx=sqrt(2)/2*(Ft1h+Ft2h-Ft3h-Ft4h);Fty=sqrt(2)/2*(-Ft1h+Ft2h+Ft3h-Ft4h);Ftz=(Ft1v+Ft2v+Ft3v+Ft4v);
    Fmx=sqrt(2)/2*(Ft1v+Ft2v-Ft3v-Ft4v)*Rp;Fmy=sqrt(2)/2*(-Ft1v+Ft2v+Ft3v-Ft4v)*Rp;Fmz=(-Ft1h-Ft2h-Ft3h-Ft4h)*Rp;
    %Ft = D*(current_Efficiency * [Ft1h, Ft2h, Ft3h, Ft4h, Ft1v, Ft2v, Ft3v, Ft4v]' + ThrustFault);  % with fault
    Ft = D*[Ft1h, Ft2h, Ft3h, Ft4h, Ft1v, Ft2v, Ft3v, Ft4v]';

    %% Simulation and parameter updating
    [T,Y] = ode45('AirShipModel30', Tspan, currentState);
    currentState = Y(length(Y),:)';
    current_px = currentState(1);
    current_py = currentState(2);
    current_pz = currentState(3);
    current_phi = currentState(4);
    current_theta = currentState(5);
    current_psi = currentState(6);
    current_u = currentState(7);
    current_v = currentState(8);
    current_w = currentState(9);
    current_p = currentState(10);
    current_q = currentState(11);
    current_r = currentState(12);
    
    %Ft = AirShipModel30_sliding_design(currentState,i,delta_t);     % 根据当前状态求出的推力
   %% YYN Control Part
    % Parameters
    k1 = 0.1;
    c2 = 0.1;
    lambdax = 0.1;
    epsilon2 = 0.1;
    
    yc = current_x_des;
    if(i==1)
        dyc = [0 0 0 0 0 0]';
        ddyc = [0 0 0 0 0 0]';
        de1 = [0 0 0 0 0 0]';
        e1 = currentState(1:6) - yc;
        alpha1 = k1*e1;
        e2 = de1 + alpha1;
        s2 = c2*e1 + e2;
    end
    
    if(i>=2)
        dyc = (yc-lastyc)/delta_t;
        ddyc = (dyc-lastdyc)/delta_t;
        e1 = currentState(1:6) - yc;
        alpha1 = k1*e1;
        de1 = currentState(7:12) - dyc;
        e2 = de1 + alpha1;
        s2 = c2*e1 + e2;
    end
    
    laste1 = e1;
    lastdyc = dyc;
    lastyc = yc;
    
    C11 = zeros(3,3);
    C12 = [0, (m+m33)*current_w, -(m+m22)*current_v+m*zG*current_p;
           -(m+m33)*current_v, 0, (m+m11)*current_u + m*zG*current_q;
           (m+m22)*current_v-m*zG*current_p, -(m+m11)*current_u-m*zG*current_q,0];
    C21 = C12;
    C22 = [0, (iz+m66)*current_r-ixz*current_p, -(iy+m55)*current_q-m*zG*current_u;
           -(iz+m66)*current_r+ixz*current_p, 0, (ix+m44)*current_p-ixz*current_r-m*zG*current_v;
           (iy+m55)*current_q+m*zG*current_u, -(ix+m44)*current_p+ixz*current_r+m*zG*current_v, 0];
    
    Corioli = [C11,C12;C21,C22];
    Damping = diag([current_u, current_v, current_w, gammax*current_p, gammay*current_q, gammaz*current_r]);
    
    Ft = M_eta*(-c2*(e2-alpha1)+M_eta\Corioli*currentState(7:12)+ M_eta\Damping*currentState(7:12)+ddyc-k1*de1)...
    -lambdax*s2 - epsilon2*sign(s2);
    %% Result Saving
    
    %求执行机构力
    FHV = pinv(D)*Ft;
    
    Ft1 = sqrt(FHV(1)^2+FHV(5)^2);
    Ft2 = sqrt(FHV(2)^2+FHV(6)^2);
    Ft3 = sqrt(FHV(3)^2+FHV(7)^2);
    Ft4 = sqrt(FHV(4)^2+FHV(8)^2);
    mu1 = atan2(FHV(1),-FHV(5));
    mu2 = atan2(FHV(2),-FHV(6));
    mu3 = atan2(FHV(3),-FHV(7));
    mu4 = atan2(FHV(4),-FHV(8));
    
    Ft_actuator = [Ft1, Ft2, Ft3, Ft4, mu1, mu2, mu3, mu4]';

    
    %inputForce(i,:) = Ft;
    %inputThrust(i,:) = pinv(CA)*Ft;
    %VP(i,:)=RIB*[current_u current_v current_w]';       % 机体坐标系的速度转换为地面坐标系下的速度
    %PP(i,:)=RIB*[current_px current_py current_pz]';    % 机体坐标系的位置转换为地面坐标下的位置
    %VA(i,:)=A*[current_p current_q current_r]';         % 机体坐标系的角速度转换为地面坐标系下的角速度
    
    current_VP=RIB*[current_u current_v current_w]';       % 机体坐标系的速度转换为地面坐标系下的速度
    current_PP=RIB*[current_px current_py current_pz]';    % 机体坐标系的位置转换为地面坐标下的位置
    current_VA=A*[current_p current_q current_r]';         % 机体坐标系的角速度转换为地面坐标系下的角速度
    
    % Print time and currentState
    fprintf(fid1,'%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n',currenttime,currentState,current_x_des);%States
    %fprintf(fid2,'%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n',currenttime,current_S);%SlidingSurfaces
    fprintf(fid3,'%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n',currenttime,Ft_actuator,Ft);%inputForces
    %fprintf(fid4,'%8.4f %8.4f %8.4f %8.4f %8.4f\n',currenttime,Vmax_est(i),Cmax_est(i),dmax_est(i),Mmax_est(i));%ParaEstimation
    fprintf(fid5,'%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n',currenttime,current_PP,current_VP,current_VA);%GroundStates

end %end the big loop

%{
 RIB=[cos(psi)*cos(theta) cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi) cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
      sin(psi)*cos(theta) sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
      -sin(theta) cos(theta)*sin(phi) cos(theta)*cos(phi)];         % 机体坐标系（速度）到惯性坐标系的转换矩阵
 A=[1 sin(phi)*tan(theta) cos(phi)*tan(theta);
    0       cos(phi)            -sin(phi);
    0 sin(phi)*sec(theta) cos(phi)*sec(theta)];                     %机体坐标系（角速度）到惯性坐标系的转换矩阵
 RBV=[cos(alpha)*cos(beta)      cos(alpha)*sin(beta) -sin(alpha);
      -sin(beta)           cos(beta)              0;
      sin(alpha)*cos(beta) sin(alpha)*sin(beta) cos(alpha)];        %气流系到机体系转换矩阵
%}
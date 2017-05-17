clear all; close all; clc;

global Fgb Fa Ft Fi current_J  ...,  %State saving
       S_norm X1_norm X2_norm current_S Vmax_est Cmax_est dmax_est Mmax_est G_norm current_Efficiency..., % Current states
       D_norm J_history dotJ ddotJ dotJ_arr ddotJ_arr x_des S_air S_air_norm D current_x_des int_se error_list Disturb
   
   %% Choose Model
 AssignModel1;
 
%% Initializing States
 current_phi = 0;
 current_theta = 0;
 current_psi = 0;
 current_px = 0;
 current_py = 0;
 current_pz = 0;
 current_u = 0;
 current_v = 0;
 current_w = 0;
 current_p = 0;
 current_q = 0;
 current_r = 0;
 currentState=[current_px current_py current_pz current_phi current_theta current_psi current_u current_v current_w current_p current_q current_r]'; 
 current_U_p=0;
 current_Efficiency = eye(8);%diag([1 1 1 1 1 1 1 1]);
 Disturb = [0 0 0 0 0 0]';
 ThrustFault = [0 0 0 0 0 0 0 0]';
 Ft_actuator = [0 0 0 0 0 0 0 0]';
 alpha = 0; 
 beta = 0;
 kappa = [2 2 2 2];
 lambda = [0 0 0 0];
 yita = [5 5 5 5];
 
 mu1=0;mu2=0;mu3=0;mu4=0;   %螺旋桨角度
 Ft1=0;Ft2=0;Ft3=0;Ft4=0;   %螺旋桨推力
 
 % Time parameters
 delta_t = 0.001;
 STEP = 40000;
 timespan = 0:delta_t:STEP*delta_t;
 %x_des = GenerateDes(timespan);
 J_history = zeros(3,36);    % Save the recent 3 history of J
 dotJ = 10*ones(6,6);
 ddotJ = 5*ones(6,6);
 dotJ_arr = zeros(2,36);
 ddotJ_arr = zeros(1,36);
 error_list = zeros(6,2);
 int_se = zeros(6,STEP);
 Vmax_est_diff = zeros(STEP,1);
 Cmax_est_diff = zeros(STEP,1);
 dmax_est_diff = zeros(STEP,1);
 Mmax_est_diff = zeros(STEP,1);
 Vmax_est = zeros(STEP,1);
 Cmax_est = zeros(STEP,1);
 dmax_est = zeros(STEP,1);
 Mmax_est = zeros(STEP,1);
 Vmax_est(1) = 0.01;
 Cmax_est(1) = 0.01;
 dmax_est(1) = 0.01;
 Mmax_est(1) = 0.01;
 
 
 %SdeSnorm = zeros(STEP,6);
 %slidingSurface = zeros(STEP,6);
 %inputForce = zeros(STEP,6);
 %inputThrust = zeros(STEP,8);
 %PP = zeros(STEP,3);
 %VP = zeros(STEP,3);
 %VA = zeros(STEP,3);
 %vwdSave = zeros(STEP,3);   % 存储风速
 
 fid1 = fopen('States.txt','wt');
 fid2 = fopen('SlidingSurfaces.txt','wt');
 fid3 = fopen('inputForces.txt','wt');
 fid4 = fopen('ParaEstimation.txt','wt');
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

    current_x_des = GenerateDes(currenttime);
    
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
		  sin(alpha)*cos(beta) sin(alpha)*sin(beta) cos(alpha)];        % Air to Body coordinate transfer matrix
    
    % Calculate the derivate of J
	current_J = blkdiag(RIB,A);
    cursor1 = mod(i+2,3)+1;
    cursor2 = mod(i+1,2)+1;
    J_history(cursor1,:) = reArrangeJ(inv(current_J));    % Could be optimized for we only need 18 elements
    
    % J is J, dotJ is J^(-1)'s dot
    
    if i>=2
        dotJ_arr(cursor2,:) = (J_history(cursor1,:)-J_history(mod(cursor1+1,3)+1,:))/delta_t;
        dotJ = inv_reArrangeJ(dotJ_arr(cursor2,:));
    end
    
    if i>=3
        ddotJ_arr = (dotJ_arr(cursor2,:)-dotJ_arr(mod(cursor2,2)+1,:))/delta_t;
        ddotJ = inv_reArrangeJ(ddotJ_arr);
    end    
    
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
	 Cx = Cx + 10*rand(1,1)^2;
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
    
    
    gammax = 500+50*randn(1,1);
    gammay = 500+50*randn(1,1);
    gammaz = 200+20*randn(1,1);
    
    
    Max=Max-gammax*current_p; May=May-gammay*current_q; Maz=-gammaz*current_r;
	%Max=Max-sign(current_p)*5000*current_p^2; May=May-sign(current_q)*5000*current_q^2; Maz=-sign(current_r)*5000*current_r^2;
    Fa=[Fax Fay Faz Max May Maz]';
    
    S_air = [Q*sref*lref*sin(Sangle) 0 0;Q*sref*lref*cos(Sangle) 0 0;0 Q*sref 0;0 0 -Q*sref*lref*cos(Sangle);0 0 Q*sref*lref*sin(Sangle);0 0 0];
    S_air_norm = norm(S_air,Inf);
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
    ThrustFault = [4 0 0 0 0.5*sin(currenttime) 0 0 0]';
    %ThrustFault = [0 0 0 0 0 0 0 0]';
    %end%Add Faults
    current_Efficiency = diag([1 1 1 1 1 1 1 1]);
    %Disturb = [0 0 0 0 0 0]';
    Disturb = [30*rand(1,1) 30*rand(1,1) 0 0.5*cos(currenttime) 0.5*sin(currenttime) 0]';
    mass(1,1) = 72 + 10.8147 + 10*sin(currenttime);
    mass(2,2) = 72 + 10.8147 + 10*sin(currenttime);
    %mass(3,3) = mass(3,3) + 5*sin(currenttime);
    %mass(4,4)= mass(4,4) + 10*sin(currenttime);
    %mass(5,5) = mass(5,5) + 10*sin(currenttime);
    %mass(6,6) = mass(6,6) + sin(currenttime);
    
    
    %Ft = D * (current_Efficiency * Ft_actuator + ThrustFault);
    Ft1h=Ft1*sin(mu1);Ft1v=-Ft1*cos(mu1);Ft2h=Ft2*sin(mu2);Ft2v=-Ft2*cos(mu2);Ft3h=Ft3*sin(mu3);Ft3v=-Ft3*cos(mu3);Ft4h=Ft4*sin(mu4);Ft4v=-Ft4*cos(mu4);
    Ftx=sqrt(2)/2*(Ft1h+Ft2h-Ft3h-Ft4h);Fty=sqrt(2)/2*(-Ft1h+Ft2h+Ft3h-Ft4h);Ftz=(Ft1v+Ft2v+Ft3v+Ft4v);
    Fmx=sqrt(2)/2*(Ft1v+Ft2v-Ft3v-Ft4v)*Rp;Fmy=sqrt(2)/2*(-Ft1v+Ft2v+Ft3v-Ft4v)*Rp;Fmz=(-Ft1h-Ft2h-Ft3h-Ft4h)*Rp;
    if(currenttime>=30);
        current_Efficiency = diag([0 1 1 1 0 1 1 1]);
    end
    Ft = D*(current_Efficiency * [Ft1h, Ft2h, Ft3h, Ft4h, Ft1v, Ft2v, Ft3v, Ft4v]' + ThrustFault);  % actual input
    
    
    
    

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
    
    if(i==12744)
        
    end
    Ft = AirShipModel30_sliding_design(currentState,i,delta_t);     % 根据当前状态求出的推力

    %% Result Saving
    
    % State Saving
    %{
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
    px(i) = current_px;
    py(i) = current_py;
    pz(i) = current_pz;
    phi(i) = current_phi;
    theta(i) = current_theta;
    psi(i) = current_psi;
    u(i) = current_u;
    v(i) = current_v;
    w(i) = current_w;
    p(i) = current_p;
    q(i) = current_q;
    r(i) = current_r;
    %}
    
    % Norm Saving
    X1_norm(i) = norm(currentState(1:6),Inf);
    X2_norm(i) = norm(currentState(7:12),Inf);
    S_norm(i) = norm(current_S,Inf);
    Fa_norm = norm(Fa,Inf);
    S_air_norm = norm(S_air,Inf);
    
    % Sliding Surface
    %slidingSurface(i,:) = current_S;
    
    for counter=1:4
        lambda(counter)=exp(-kappa(counter)*currenttime);
    end
    
    if i == 100
        
    end
    %Vmax_est_diff(i) = -(lambda(1)^2)*Vmax_est(i)+yita(1)*D_norm*S_norm(i);
    %Cmax_est_diff(i) = -(lambda(2)^2)*Cmax_est(i)+yita(2)*S_air_norm*S_norm(i);
    %dmax_est_diff(i) = -(lambda(3)^2)*dmax_est(i)+yita(3)*S_norm(i);
    %Mmax_est_diff(i) = -(lambda(4)^2)*Mmax_est(i)+yita(4)*G_norm*S_norm(i);

    Vmax_est_diff(i) = -(lambda(1)^2-0.001)*Vmax_est(i)+yita(1)*D_norm*S_norm(i);
    Cmax_est_diff(i) = -(lambda(2)^2-0.001)*Cmax_est(i)+yita(2)*S_air_norm*S_norm(i);
    dmax_est_diff(i) = -(lambda(3)^2-0.001)*dmax_est(i)+yita(3)*S_norm(i);
    Mmax_est_diff(i) = -(lambda(4)^2-0.001)*Mmax_est(i)+yita(4)*G_norm*S_norm(i);
    
    if i==1
        Vmax_est(2) = Vmax_est(1)+Vmax_est_diff(1)*delta_t;
        Cmax_est(2) = Cmax_est(1)+Cmax_est_diff(1)*delta_t;
        dmax_est(2) = dmax_est(1)+dmax_est_diff(1)*delta_t;
        Mmax_est(2) = Mmax_est(1)+Mmax_est_diff(1)*delta_t;
    end
    
    if i>=2
        int_tspan = delta_t:delta_t:delta_t*i;
        Vmax_est(i+1) = Vmax_est(i)+Vmax_est_diff(i)*delta_t;%trapz(int_tspan,Vmax_est_diff(1:i));
        Cmax_est(i+1) = Cmax_est(i)+Cmax_est_diff(i)*delta_t;%trapz(int_tspan,Cmax_est_diff(1:i));
        dmax_est(i+1) = dmax_est(i)+dmax_est_diff(i)*delta_t;%trapz(int_tspan,dmax_est_diff(1:i));
        Mmax_est(i+1) = Mmax_est(i)+Mmax_est_diff(i)*delta_t;%trapz(int_tspan,Mmax_est_diff(1:i));
    end 

    
    
    
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
    fprintf(fid2,'%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n',currenttime,current_S);%SlidingSurfaces
    fprintf(fid3,'%8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n',currenttime,Ft,Ft_actuator);%inputForces
    fprintf(fid4,'%8.4f %8.4f %8.4f %8.4f %8.4f\n',currenttime,Vmax_est(i),Cmax_est(i),dmax_est(i),Mmax_est(i));%ParaEstimation
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
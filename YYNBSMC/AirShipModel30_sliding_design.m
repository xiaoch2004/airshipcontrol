function f = AirShipModel30_sliding_design(x,i,delta_t)
global Fgb Fi current_S Vmax_est Cmax_est dmax_est Mmax_est G_norm G ..., % Current states
       F_norm S_norm D_norm dotJ ddotJ current_x_des D current_J S_air_norm int_se error_list%Fa Fi Fgb
% global current_phi current_theta current_psi current_u current_v current_w current_p current_q current_r alpha
% Structure of x: px py pz phi theta psi u v w p q r

miu = 0.001;
sigma = 8;
alpha_con = 0.5;
beta_con = 0.5;
THETA = diag([1 1 1 1 1 1 1 1]);
V_bar = zeros(8,1);
epsilon = 0.02;
cursor = mod(i+1,2)+1;
dot_error_init = zeros(6,1);
k1 = 500;
k2 = 1000;
k3 = 1;
k4 = 10000;


error_list(:,cursor) = x(1:6)-current_x_des;
%error_list(6,cursor) = 0;
    if i==1
        dot_error = dot_error_init;
    elseif i>=2
        dot_error = (error_list(:,cursor)-error_list(:,3-cursor))/delta_t;
    end
int_se(:,i) = -alpha_con*dot_error-beta_con*error_list(:,cursor);
int_tspan = delta_t:delta_t:delta_t*i;

    if i==1
        current_S = 0.5*ones(6,1);
    elseif i>=2
        for k=1:1:6  
            current_S(k) = dot_error(k) - dot_error_init(k) - trapz(int_tspan,int_se(k,1:i));
        end
    end

S_norm(i) = norm(current_S,Inf);
DSS_con = (D'*current_S);

G = ddotJ*error_list(:,cursor)+dotJ*current_J*x(7:12)+alpha_con*dotJ*error_list(:,cursor)+alpha_con*x(7:12)+beta_con*inv(current_J)*error_list(:,cursor);
G_norm = norm(G,Inf);
F_norm = norm(Fgb+Fi,Inf);

%f = mass*(-tao*S-sigma*sign(S))-mass*x(7:12)-Fa-Fi-Fgb;
%f = - tao*current_S - sigma*sign(current_S) - current_Up;
V = -(F_norm*DSS_con)/(sigma*S_norm(i)+epsilon)-(D_norm*Vmax_est(i)*DSS_con)/(k1*S_norm(i)+epsilon)-...
    (S_air_norm*Cmax_est(i)*DSS_con)/(k2*S_norm(i)+epsilon)-dmax_est(i)*DSS_con/(k3*S_norm(i)+epsilon)-...
    -(G_norm*Mmax_est(i)*DSS_con)/(k4*S_norm(i)+epsilon)-miu*DSS_con/(S_norm(i)+ epsilon);
%V = -(F_norm*DSS_con)/(sigma*S_norm(i)+epsilon)-(D_norm*Vmax_est(i)*DSS_con)/(k1*S_norm(i)+epsilon)-(S_air_norm*Cmax_est(i)*DSS_con)/(k2*S_norm(i)+epsilon)-dmax_est(i)*DSS_con/(k3*S_norm(i)+epsilon)-(G_norm*Mmax_est(i)*DSS_con)/(k4*S_norm(i)+epsilon)-miu*DSS_con/(S_norm(i)+ epsilon);

f = D*(THETA*V+V_bar);

end


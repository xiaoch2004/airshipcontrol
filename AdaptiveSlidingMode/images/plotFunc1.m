
%% Locus Fig
figure(1);
plot(S(:,2),S(:,3),data(:,3),data(:,5),'--',Y(:,2),Y(:,3),'--','LineWidth',1.5);xlabel('X/m');ylabel('Y/m');
legend('our method','Method of Ding','Method of Yang','location','southeast');
hold on;
plot(0,0,'o');
plot(9.7,9.7,'o');
title('Horizontal locus compare of 3 methods');
text(-0.5,-1,'Start Location');
text(7,11,'Desired Location');
hold off;
print -depsc f1locus
%% Psi angle Fig
figure(2);
plot(S(:,1),S(:,7),data(:,1),data(:,11),'--',Y(:,1),Y(:,7),'--',S(:,1),zeros(length(S(:,1)),1),':','LineWidth',1.5);xlabel('Time/s');ylabel('Angle/rad');
legend('our method','Method of Ding','Method of Yang','Desired Yaw angle','location','southwest');
title('Time response of \psi (yaw angle)');
print -depsc f1psi
%% Input fig1
figure(3);
title('Thrusts of 4 propellers');
subplot(4,1,1)
plot(inputForces(:,1),inputForces(:,8),'LineWidth',1.5);xlabel('Time/s');ylabel('Thrust1/N');
subplot(4,1,2)
plot(inputForces(:,1),inputForces(:,9),'LineWidth',1.5);xlabel('Time/s');ylabel('Thrust2/N');
subplot(4,1,3)
plot(inputForces(:,1),inputForces(:,10),'LineWidth',1.5);xlabel('Time/s');ylabel('Thrust3/N');
subplot(4,1,4)
plot(inputForces(:,1),inputForces(:,11),'LineWidth',1.5);xlabel('Time/s');ylabel('Thrust4/N');
print -depsc f1input1mine
%% Input fig2
figure(4);
subplot(4,1,1)
title('Angle of 4 propellers');
plot(inputForces(:,1),inputForces(:,12),'LineWidth',1.5);xlabel('Time/s');ylabel('Angle/rad');
subplot(4,1,2)
plot(inputForces(:,1),inputForces(:,13),'LineWidth',1.5);xlabel('Time/s');ylabel('Angle/rad');
subplot(4,1,3)
plot(inputForces(:,1),inputForces(:,14),'LineWidth',1.5);xlabel('Time/s');ylabel('Angle/rad');
subplot(4,1,4)
plot(inputForces(:,1),inputForces(:,15),'LineWidth',1.5);xlabel('Time/s');ylabel('Angle/rad');
print -depsc f1input2mine


%% Height compare
figure(5);
b = plot(S(:,1),S(:,4),data(:,1),data(:,7),'--',Y(:,1),Y(:,4),'--',S(:,1),zeros(length(S(:,1)),1),':','LineWidth',1.5);xlabel('Time/s');ylabel('Height/m');
hold on;
legend('our method','Method of Ding','Method of Yang','Desired Height','location','southwest');
title('Time response of Airship height');
print -depsc f1height
%{
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
plot(S(:,1),S(:,7),data(:,1),data(:,11),'--',Y(:,1),Y(:,7),'--','LineWidth',1.5);xlabel('Time/s');ylabel('Angle/rad');
legend('our method','Method of Ding','Method of Yang','location','southwest');
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
%}
%% Locus Fig
figure(1);
b = plot(S(:,2),S(:,3),data(:,3),data(:,5),'--',Y(:,2),Y(:,3),'--','LineWidth',1.5);xlabel('X/m');ylabel('Y/m');
hold on;
plot(0,0,'o');  %start point
plot(9.7,9.7,'o');  %end point
f = plot([Y(30000,2),S(30000,2),data(3000,3)],[Y(30000,3),S(30000,3),data(3000,5)],'k.','MarkerSize',20);
axis([-2 15 -5 12]);
title('Horizontal locus compare of 3 methods');
text(-0.5,-1,'Start Location');
text(10,10,'Desired Location');
legend([b;f],'our method','Method of Ding','Method of Yang','Thrust stuck position','location','northwest');
hold off;
print -depsc f2locus
%% Psi angle Fig
figure(2);
b = plot(S(:,1),S(:,7),data(:,1),data(:,9),'--',Y(:,1),Y(:,7),'--',S(:,1),S(:,19),'--','LineWidth',1.5);xlabel('Time/s');ylabel('Angle/rad');
hold on;
f = plot([30,30],[-3,2],':k','LineWidth',1.5);
hold off;
legend([b;f],'our method','Method of Ding','Method of Yang','Desired yaw angle','Thrust Stuck','location','southwest');
title('Time response of \psi (yaw angle)');
print -depsc f2psi
%% Input fig1
figure(3);
title('Thrusts of 4 propellers');
subplot(4,1,1)
plot(I(:,1),I(:,8),'LineWidth',1.5);xlabel('Time/s');ylabel('Thrust1/N');
subplot(4,1,2)
plot(inputForces(:,1),inputForces(:,9),'LineWidth',1.5);xlabel('Time/s');ylabel('Thrust2/N');
subplot(4,1,3)
plot(inputForces(:,1),inputForces(:,10),'LineWidth',1.5);xlabel('Time/s');ylabel('Thrust3/N');
subplot(4,1,4)
plot(inputForces(:,1),inputForces(:,11),'LineWidth',1.5);xlabel('Time/s');ylabel('Thrust4/N');
print -depsc f2input1mine
%% Input fig2
figure(4);
subplot(4,1,1)
title('Angle of 4 propellers');
plot(I(:,1),I(:,12),'LineWidth',1.5);xlabel('Time/s');ylabel('Angle/rad');
subplot(4,1,2)
plot(inputForces(:,1),inputForces(:,13),'LineWidth',1.5);xlabel('Time/s');ylabel('Angle/rad');
subplot(4,1,3)
plot(inputForces(:,1),inputForces(:,14),'LineWidth',1.5);xlabel('Time/s');ylabel('Angle/rad');
subplot(4,1,4)
plot(inputForces(:,1),inputForces(:,15),'LineWidth',1.5);xlabel('Time/s');ylabel('Angle/rad');
print -depsc f2input2mine

%% Height compare
figure(5);
b = plot(S(:,1),S(:,4),data(:,1),data(:,7),'--',Y(:,1),Y(:,4),'--',S(:,1),zeros(length(S(:,1)),1),':','LineWidth',1.5);xlabel('Time/s');ylabel('Height/m');
hold on;
f = plot([30,30],[-1.5,0.6],':k','LineWidth',1.5);
axis([0,40,-1.5,0.6]);
hold off;
legend([b;f],'our method','Method of Ding','Method of Yang','Desired Height','Thrust Stuck','location','southwest');
title('Time response of Airship height');
print -depsc f2height
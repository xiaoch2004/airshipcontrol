Ay = load('GroundStates.txt');
By = load('States.txt');
Cy = load('inputForces.txt');

figure(1);
subplot(2,2,1);
plot(Ay(:,1),Ay(:,2),'-r','LineWidth',1.5);xlabel('time/s');ylabel('Cartesian Positions(m)');hold on;
plot(Ay(:,1),Ay(:,3),'-g','LineWidth',1.5);hold on;
plot(Ay(:,1),Ay(:,4),'-b','LineWidth',1.5);
legend('X Position','Y Position','Z Position');
%print -depsc plot1.eps
subplot(2,2,2);
plot(By(:,1),By(:,5),'-r','LineWidth',1.5);xlabel('time/s');ylabel('Euler Attitudes(rad)');hold on;
plot(By(:,1),By(:,6),'-g','LineWidth',1.5);hold on;
plot(By(:,1),By(:,7),'-b','LineWidth',1.5);
h = legend('$\phi$ Angle','$\theta$ Angle','$\psi$ Angle'); set(h,'interpreter','latex');
%print -depsc plot2.eps
subplot(2,2,3);
plot(Ay(:,1),Ay(:,5),'-r','LineWidth',1.5);xlabel('time/s');ylabel('Linear velocities(m/s)');hold on;
plot(Ay(:,1),Ay(:,6),'-g','LineWidth',1.5);hold on;
plot(Ay(:,1),Ay(:,7),'-b','LineWidth',1.5);
legend('u','v','w');
subplot(2,2,4);
plot(Ay(:,1),Ay(:,8),'-r','LineWidth',1.5);xlabel('time/s');ylabel('Angular velocities(rad/s)');hold on;
plot(Ay(:,1),Ay(:,9),'-g','LineWidth',1.5);hold on;
plot(Ay(:,1),Ay(:,10),'-b','LineWidth',1.5);
legend('p','q','r');
print -depsc YYNstates.eps

figure(2);
subplot(4,1,1);
plot(Cy(:,1),Cy(:,2),'LineWidth',1.5);xlabel('time/s');ylabel('Thrust1/N');%ylim([-150 150]);
subplot(4,1,2);
plot(Cy(:,1),Cy(:,3),'LineWidth',1.5);xlabel('time/s');ylabel('Thrust2/N');%ylim([-150 150]);
subplot(4,1,3);
plot(Cy(:,1),Cy(:,4),'LineWidth',1.5);xlabel('time/s');ylabel('Thrust3/N');%ylim([-150 150]);
subplot(4,1,4);
plot(Cy(:,1),Cy(:,5),'LineWidth',1.5);xlabel('time/s');ylabel('Thrust4/N');%ylim([-150 150]);
print -depsc YYNthrust.eps

figure(3);
subplot(4,1,1);
plot(Cy(:,1),Cy(:,6),'LineWidth',1.5);xlabel('time/s');ylabel('Thrust1/rad');%ylim([-150 150]);
subplot(4,1,2);
plot(Cy(:,1),Cy(:,7),'LineWidth',1.5);xlabel('time/s');ylabel('Thrust2/rad');%ylim([-150 150]);
subplot(4,1,3);
plot(Cy(:,1),Cy(:,8),'LineWidth',1.5);xlabel('time/s');ylabel('Thrust3/rad');%ylim([-150 150]);
subplot(4,1,4);
plot(Cy(:,1),Cy(:,9),'LineWidth',1.5);xlabel('time/s');ylabel('Thrust4/rad');%ylim([-150 150]);
print -depsc YYNangle.eps





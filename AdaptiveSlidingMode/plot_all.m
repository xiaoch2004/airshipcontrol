States = load('States.txt');
groundStates = load('groundStates.txt');
slidingSurfaces = load('SlidingSurfaces.txt');
inputForces = load('inputForces.txt');
para = load('ParaEstimation.txt');


figure(1);
subplot(2,1,1);
plot(States(:,1),States(:,2),'k',States(:,1),States(:,14),'--b');xlabel('time/s');ylabel('X/m');
title('Time response of X position under Sliding Mode method');legend('Actual','Desired','Location','southeast')
subplot(2,1,2);
plot(States(:,1),States(:,7),'k',States(:,1),States(:,19),'--b');xlabel('time/s');ylabel('\psi/rad');
title('Time response of yaw angle under Sliding Mode method');legend('Actual','Desired','Location','southeast')
print -depsc XandPsi-mine1

figure(2);
subplot(2,1,1);
plot(States(:,1),States(:,3),'k',States(:,1),States(:,15),'--b');xlabel('time/s');ylabel('Y/m');
title('Time response of Y position under Sliding Mode method');legend('Actual','Desired','Location','southeast')
subplot(2,1,2);
plot(States(:,1),States(:,4),'k',States(:,1),States(:,16),'--b');xlabel('time/s');ylabel('Z/m');
title('Time response of Z position under Sliding Mode method');legend('Actual','Desired','Location','southeast')
print -depsc YandZ-mine1

figure(3);
subplot(2,1,1);
plot(States(:,1),States(:,5),'k',States(:,1),States(:,17),'--b');xlabel('time/s');ylabel('\phi/rad');
title('Time response of roll angle under Sliding Mode method');legend('Actual','Desired')
subplot(2,1,2);
plot(States(:,1),States(:,6),'k',States(:,1),States(:,18),'--b');xlabel('time/s');ylabel('\theta/rad');
title('Time response of pitch angle under Sliding Mode method');legend('Actual','Desired')
print -depsc Phiandtheta-mine1



figure(3);
subplot(4,2,1);
plot(inputForces(:,1),inputForces(:,8),'-b','LineWidth',2);xlabel('time/s');ylabel('Ft1/N');legend('Ft1');%ylim([-150 150]);
subplot(4,2,3);
plot(inputForces(:,1),inputForces(:,9),'-b','LineWidth',2);xlabel('time/s');ylabel('Ft2/N');legend('Ft2');%ylim([-150 150]);
subplot(4,2,5);
plot(inputForces(:,1),inputForces(:,10),'-b','LineWidth',2);xlabel('time/s');ylabel('Ft3/N');legend('Ft3');%ylim([-150 150]);
subplot(4,2,7);
plot(inputForces(:,1),inputForces(:,11),'-b','LineWidth',2);xlabel('time/s');ylabel('Ft4/N');legend('Ft4');%ylim([-150 150]);
subplot(4,2,2);
plot(inputForces(:,1),inputForces(:,12),'-b','LineWidth',2);xlabel('time/s');ylabel('mu1/N');legend('Mu1');%ylim([-150 150]);
subplot(4,2,4);
plot(inputForces(:,1),inputForces(:,13),'-b','LineWidth',2);xlabel('time/s');ylabel('mu1/N');legend('Mu2');%ylim([-150 150]);
subplot(4,2,6);
plot(inputForces(:,1),inputForces(:,14),'-b','LineWidth',2);xlabel('time/s');ylabel('mu1/N');legend('Mu3');%ylim([-150 150]);
subplot(4,2,8);
plot(inputForces(:,1),inputForces(:,15),'-b','LineWidth',2);xlabel('time/s');ylabel('mu1/N');legend('Mu4');%ylim([-150 150]);


figure(4);
subplot(3,2,1);
plot(slidingSurfaces(:,1),slidingSurfaces(:,2),'-r','LineWidth',2);xlabel('time/s');legend('S_1');%ylim([-1 3]);
subplot(3,2,3);
plot(slidingSurfaces(:,1),slidingSurfaces(:,3),'-g','LineWidth',2);xlabel('time/s');legend('S_2');%ylim([-1 3]);
subplot(3,2,5);
plot(slidingSurfaces(:,1),slidingSurfaces(:,4),'-b','LineWidth',2);xlabel('time/s');legend('S_3');%ylim([-1 3]);
subplot(3,2,2);
plot(slidingSurfaces(:,1),slidingSurfaces(:,5),'-r','LineWidth',2);xlabel('time/s');legend('S_4');%ylim([-1 3]);
subplot(3,2,4);
plot(slidingSurfaces(:,1),slidingSurfaces(:,6),'-g','LineWidth',2);xlabel('time/s');legend('S_5');%ylim([-1 3]);
subplot(3,2,6);
plot(slidingSurfaces(:,1),slidingSurfaces(:,7),'-b','LineWidth',2);xlabel('time/s');legend('S_6');%ylim([-1 3]);

figure(5);
subplot(2,2,1);
plot(para(:,1),para(:,2),'LineWidth',2);xlabel('time/s');txt = texlabel('V_max'); text(10,10,txt);
subplot(2,2,2);
plot(para(:,1),para(:,3),'LineWidth',2);xlabel('time/s');txt = texlabel('c_max'); text(10,10,txt);
subplot(2,2,3);
plot(para(:,1),para(:,4),'LineWidth',2);xlabel('time/s');txt = texlabel('d_max'); text(10,10,txt);
subplot(2,2,4);
plot(para(:,1),para(:,5),'LineWidth',2);xlabel('time/s');txt = texlabel('M_max'); text(10,10,txt);

%}


%{
figure(1);
subplot(2,2,1);
plot(groundStates(:,1),groundStates(:,2),'-r','LineWidth',1.5);xlabel('time/s');ylabel('Cartesian Positions(m)');ylim([-5 5]);hold on;
plot(groundStates(:,1),groundStates(:,3),'-g','LineWidth',1.5);hold on;
plot(groundStates(:,1),groundStates(:,4),'-b','LineWidth',1.5);
legend('X Position','Y Position','Z Position');
print -depsc position
subplot(2,2,2);
plot(States(:,1),States(:,5),'-r','LineWidth',1.5);xlabel('time/s');ylabel('Euler Attitudes(rad)');ylim([-pi pi]);hold on;
plot(States(:,1),States(:,6),'-g','LineWidth',1.5);hold on;
plot(States(:,1),States(:,7),'-b','LineWidth',1.5);
h = legend('$\phi$ Angle','$\theta$ Angle','$\psi$ Angle'); set(h,'interpreter','latex');
print -depsc angle
subplot(2,2,3);
plot(groundStates(:,1),groundStates(:,5),'-r','LineWidth',1.5);xlabel('time/s');ylabel('Linear velocities(m/s)');ylim([-2 4]);hold on;
plot(groundStates(:,1),groundStates(:,6),'-g','LineWidth',1.5);hold on;
plot(groundStates(:,1),groundStates(:,7),'-b','LineWidth',1.5);
legend('u','v','w');
print -depsc groundVelocities
subplot(2,2,4);
plot(States(:,1),States(:,11),'-r','LineWidth',1.5);xlabel('time/s');ylabel('Angular velocities(rad/s)');ylim([-1 1]);hold on;
plot(States(:,1),States(:,12),'-g','LineWidth',1.5);hold on;
plot(States(:,1),States(:,13),'-b','LineWidth',1.5);
legend('p','q','r');
print -depsc groundAngularVelocities

figure(2);
subplot(3,2,1);
plot(inputForces(:,1),inputForces(:,2),'-r','LineWidth',2);xlabel('time/s');ylabel('x Force/N');legend('x Force');%ylim([-150 150]);
subplot(3,2,2);
plot(inputForces(:,1),inputForces(:,3),'-g','LineWidth',2);xlabel('time/s');ylabel('y Force/N');legend('y Force');%ylim([-150 150]);
subplot(3,2,3);
plot(inputForces(:,1),inputForces(:,4),'-b','LineWidth',2);xlabel('time/s');ylabel('z Force/N');legend('z Force');%ylim([-200 150]);
subplot(3,2,4);
plot(inputForces(:,1),inputForces(:,5),'-r','LineWidth',2);xlabel('time/s');ylabel('x Moment/kgm^2');legend('x Moment');%ylim([-150 150]);
subplot(3,2,5);
plot(inputForces(:,1),inputForces(:,6),'-g','LineWidth',2);xlabel('time/s');ylabel('y Moment/kgm^2');legend('y Moment');%ylim([-150 150]);
subplot(3,2,6);
plot(inputForces(:,1),inputForces(:,7),'-b','LineWidth',2);xlabel('time/s');ylabel('z Moment/kgm^2');legend('z Moment');%ylim([-150 150]);
print -depsc inputForces


figure(3);
subplot(4,2,1);
plot(inputForces(:,1),inputForces(:,8),'-b','LineWidth',2);xlabel('time/s');ylabel('Ft1/N');legend('Ft1');%ylim([-150 150]);
subplot(4,2,3);
plot(inputForces(:,1),inputForces(:,9),'-b','LineWidth',2);xlabel('time/s');ylabel('Ft2/N');legend('Ft2');%ylim([-150 150]);
subplot(4,2,5);
plot(inputForces(:,1),inputForces(:,10),'-b','LineWidth',2);xlabel('time/s');ylabel('Ft3/N');legend('Ft3');%ylim([-150 150]);
subplot(4,2,7);
plot(inputForces(:,1),inputForces(:,11),'-b','LineWidth',2);xlabel('time/s');ylabel('Ft4/N');legend('Ft4');%ylim([-150 150]);
subplot(4,2,2);
plot(inputForces(:,1),inputForces(:,12),'-b','LineWidth',2);xlabel('time/s');ylabel('mu1/rad');legend('Mu1');%ylim([-150 150]);
subplot(4,2,4);
plot(inputForces(:,1),inputForces(:,13),'-b','LineWidth',2);xlabel('time/s');ylabel('mu1/rad');legend('Mu2');%ylim([-150 150]);
subplot(4,2,6);
plot(inputForces(:,1),inputForces(:,14),'-b','LineWidth',2);xlabel('time/s');ylabel('mu1/rad');legend('Mu3');%ylim([-150 150]);
subplot(4,2,8);
plot(inputForces(:,1),inputForces(:,15),'-b','LineWidth',2);xlabel('time/s');ylabel('mu1/rad');legend('Mu4');%ylim([-150 150]);
print -depsc Thrusts




figure(4);
subplot(3,2,1);
plot(slidingSurfaces(:,1),slidingSurfaces(:,2),'-r','LineWidth',2);xlabel('time/s');legend('S_1');ylim([-1 3]);
subplot(3,2,3);
plot(slidingSurfaces(:,1),slidingSurfaces(:,3),'-g','LineWidth',2);xlabel('time/s');legend('S_2');ylim([-1 3]);
subplot(3,2,5);
plot(slidingSurfaces(:,1),slidingSurfaces(:,4),'-b','LineWidth',2);xlabel('time/s');legend('S_3');ylim([-1 3]);
subplot(3,2,2);
plot(slidingSurfaces(:,1),slidingSurfaces(:,5),'-r','LineWidth',2);xlabel('time/s');legend('S_4');ylim([-1 3]);
subplot(3,2,4);
plot(slidingSurfaces(:,1),slidingSurfaces(:,6),'-g','LineWidth',2);xlabel('time/s');legend('S_5');ylim([-1 3]);
subplot(3,2,6);
plot(slidingSurfaces(:,1),slidingSurfaces(:,7),'-b','LineWidth',2);xlabel('time/s');legend('S_6');ylim([-1 3]);
print -depsc slidingSurfaces

%}
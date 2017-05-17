pp=load('data.txt');
jj=load('F_NU.txt');

figure(1)

subplot(2,1,1)
plot(pp(:,1),pp(:,2),'b--',pp(:,1),pp(:,3),'k');
title('Time response of X position under Adaptive Backstepping method');
ylabel('X/m')
xlabel('time/s')
legend('Desired','Actual','Location','southeast')
%figure(2)
subplot(2,1,2)
plot(pp(:,1),pp(:,8),'b--',pp(:,1),pp(:,9),'k');
title('Time response of yaw angle under Adaptive Backstepping method');
ylabel('\psi/rad')
xlabel('time/s')
legend('Desired','Actual')
print -depsc XandPsi

figure(2)
subplot(2,1,2)
plot(pp(:,1),pp(:,6),'b--',pp(:,1),pp(:,7),'k');
title('Time response of Z position under Adaptive Backstepping method');
ylabel('Z/m')
xlabel('time/s')
legend('Desired','Actual','Location','southeast')
subplot(2,1,1)
plot(pp(:,1),pp(:,4),'b--',pp(:,1),pp(:,5),'k');
title('Time response of Y position under Adaptive Backstepping method');
ylabel('Y/m')
xlabel('time/s')
legend('Desired','Actual','Location','southeast')
print -depsc YandZ


figure(3)
subplot(2,1,1)
plot(pp(:,1),pp(:,10),'b--',pp(:,1),pp(:,11),'k');
title('Time response of roll angle under Adaptive Backstepping method');
ylabel('\phi/rad')
xlabel('time/s')
legend('Desired','Actual')

subplot(2,1,2)
plot(pp(:,1),pp(:,12),'b--',pp(:,1),pp(:,13),'k');
title('Time response of pitch angle under Adaptive Backstepping method');
ylabel('\theta/rad')
xlabel('time/s')
legend('Desired','Actual')
print -depsc PhiandTheta

%{
figure(4)
subplot(2,2,1)
plot(pp(:,1),pp(:,2)-pp(:,3),'k');
ylabel('Error x(m)')
xlabel('time (s)')

subplot(2,2,2)
plot(pp(:,1),pp(:,4)-pp(:,5),'k');
ylabel('Error y(m)')
xlabel('time (s)')

subplot(2,2,3)
plot(pp(:,1),pp(:,6)-pp(:,7),'k');
ylabel('Error z(m)')
xlabel('time (s)')

subplot(2,2,4)
plot(pp(:,1),pp(:,8)-pp(:,9),'k');
ylabel('Error psi(rad)')
xlabel('time (s)')

figure(5)
subplot(2,2,1)
plot(jj(:,1),jj(:,2),'k');
ylabel('Ft1(N)')
xlabel('time (s)')

subplot(2,2,2)
plot(jj(:,1),jj(:,3),'k');
ylabel('Ft2(N)')
xlabel('time (s)')

subplot(2,2,3)
plot(jj(:,1),jj(:,4),'k');
ylabel('Ft3(N)')
xlabel('time (s)')

subplot(2,2,4)
plot(jj(:,1),jj(:,5),'k');
ylabel('Ft4(N)')
xlabel('time (s)')

figure(6)
subplot(2,2,1)
plot(jj(:,1),jj(:,6),'k');
ylabel('mu1(rad)')
xlabel('time (s)')

subplot(2,2,2)
plot(jj(:,1),jj(:,7),'k');
ylabel('mu2(rad)')
xlabel('time (s)')

subplot(2,2,3)
plot(jj(:,1),jj(:,8),'k');
ylabel('mu3(rad)')
xlabel('time (s)')

subplot(2,2,4)
plot(jj(:,1),jj(:,9),'k');
ylabel('mu4(rad)')
xlabel('time (s)')

figure(7)
subplot(3,2,1)
plot(pp(:,1),pp(:,14),'k');
ylabel('Cx')
xlabel('time (s)')
grid on
subplot(3,2,2)
plot(pp(:,1),pp(:,15),'k');
ylabel('Cy')
xlabel('time (s)')
grid on
subplot(3,2,3)
plot(pp(:,1),pp(:,16),'k');
ylabel('Cz')
xlabel('time (s)')
grid on
subplot(3,2,4)
plot(pp(:,1),pp(:,17),'k');
ylabel('Cmx')
xlabel('time (s)')
grid on
subplot(3,2,5)
plot(pp(:,1),pp(:,18),'k');
ylabel('Cmy')
xlabel('time (s)')
grid on
subplot(3,2,6)
plot(pp(:,1),pp(:,19),'k');
ylabel('Cmz')
xlabel('time (s)')
grid on
figure(8)
subplot(3,2,1)
plot(jj(:,1),jj(:,10));
ylabel('Ftx')
xlabel('time (s)')
subplot(3,2,2)
plot(jj(:,1),jj(:,11));
ylabel('Fty')
xlabel('time (s)')
subplot(3,2,3)
plot(jj(:,1),jj(:,12));
ylabel('Ftz')
xlabel('time (s)')
subplot(3,2,4)
plot(jj(:,1),jj(:,13));
ylabel('Fmx')
xlabel('time (s)')
subplot(3,2,5)
plot(jj(:,1),jj(:,14));
ylabel('Fmy')
xlabel('time (s)')
subplot(3,2,6)
plot(jj(:,1),jj(:,15));
ylabel('Fmz')
xlabel('time (s)')
%}
% plot(jj(:,1),jj(:,10:15));
% ylabel('a2')
% xlabel('time (s)')

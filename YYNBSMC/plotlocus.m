plot(By(:,2),By(:,3),'--b');axis([-5 25 -5 12]);xlabel('X/m');ylabel('Y/m');title('Airship planar locus compare');
hold on;
plot(B(:,2),B(:,3),'-r');legend('Method of Yang','Our method');
c = scatter([0 5],[0,1.5],'filled');
text(5,2.5,'Start position:(5,1.5)');
text(0,-1,'Desired stable position:(0,0)');
print -depsc locuscompare.eps
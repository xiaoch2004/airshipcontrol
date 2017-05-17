function windList = GenerateWind(STEP, dim, delta_t)
%GENERATEWIND Summary of this function goes here
%   Detailed explanation goes here
windList = zeros(dim,STEP);
T = zeros(1,STEP);
for i=1:1:STEP
    current_time = delta_t*i;
    T(i) = current_time;
    if current_time>50
        windList(1,i)=0;
        windList(2,i)=0;
        windList(3,i)=0;
    else
        windList(1,i)=-0.005*current_time^2 + 0.25*current_time;
        windList(2,i)=0.005*current_time^2 - 0.25*current_time;
        windList(3,i)=0.002*current_time^2 - 0.1*current_time;
    end
end
figure(6);
subplot(3,1,1);
plot(T,windList(1,:),'-r','LineWidth',2);xlabel('time/s');ylabel('x Wind/(m/s)');ylim([-1,4]);
subplot(3,1,2);
plot(T,windList(2,:),'-g','LineWidth',2);xlabel('time/s');ylabel('y Wind/(m/s)');ylim([-4,1]);
subplot(3,1,3);
plot(T,windList(3,:),'-b','LineWidth',2);xlabel('time/s');ylabel('z Wind/(m/s)');ylim([-2,2]);
print -depsc windPlot
end


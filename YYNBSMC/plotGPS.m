function plotGPS(data)

x = data(:,1);
y = data(:,2);

figure;hold on;

axis([min(x)-1,max(x)+1,min(y)-1,max(y)+1]);

for i = 1:1:(size(data,1)-1)
    %pause(0.00001);
    plot(x(i:i+1),y(i:i+1),'r');
end
end
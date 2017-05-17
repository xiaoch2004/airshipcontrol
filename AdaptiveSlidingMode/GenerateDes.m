function Des = GenerateDes( timepoint )
%GENERATEDES Summary of this function goes here
%   Detailed explanation goes here
%delta_t = timeSpan(2)-timeSpan(1);

%STEP = length(timeSpan);
Des = zeros(6,1);

if (timepoint<=40) 
    Des(1) = 10*(2*sigmf(timepoint,[0.1,0])-1);   % Go up
    Des(2) = 10*(2*sigmf(timepoint,[0.1,0])-1);%-3*(2*sigmf(timepoint,[0.2,0])-1);   %Set desired y position
    Des(3) = 0;%-2*(2*sigmf(timepoint,[0.2,0])-1);   %Set desired z position
    Des(4) = 0;   %Set desired phi position
    Des(5) = 0;   %Set desired theta position
    Des(6) = (pi/2)*(2*sigmf(timepoint,[0.2,0])-1);   %Set desired psi position
end

%{
if(timepoint < 20)
    Des(1) = 1;   %Set desired x position
    Des(2) = -1;   %Set desired y position
    Des(3) = 1;   %Set desired z position
    Des(4) = 0;   %Set desired phi position
    Des(5) = 0;   %Set desired theta position
    Des(6) = pi/6;   %Set desired psi position
elseif(timepoint<=25)
    Des(1,:) = timepoint - 19;   %Set desired x position
    Des(2,:) = -1;   %Set desired y position
    Des(3,:) = 1;   %Set desired z position
    Des(4,:) = 0;   %Set desired phi position
    Des(5,:) = 0;   %Set desired theta position
    Des(6,:) = pi/6;   %Set desired psi position
else
    Des(1,:) = 6;   %Set desired x position
    Des(2,:) = -1;   %Set desired y position
    Des(3,:) = 1;   %Set desired z position
    Des(4,:) = 0;   %Set desired phi position
    Des(5,:) = 0;   %Set desired theta position
    Des(6,:) = pi/6;   %Set desired psi position   
end
%}
    
end


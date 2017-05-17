function output = reArrangeJ( J )
%REARRANGEJ Summary of this function goes here
%   Detailed explanation goes here
M = size(J);
output = zeros(1,36);
if M(1)==6 && M(2)==6
    for i=1:1:36
        output(i)=J(i);
    end
else
    disp(sprintf('The size of J did not match function reArrange()'));
end
end

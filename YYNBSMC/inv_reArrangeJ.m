function J = inv_reArrangeJ( output )
%INV_REARRANGEJ Summary of this function goes here
%   Detailed explanation goes here
M = size(output);
J = zeros(6,6);
if M(1)==1 && M(2)==36
    for i=1:1:36
        J(i)=output(i);
    end
else
    disp(sprintf('The size of J did not match function inv_reArrange()'));
end
end


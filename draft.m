% function [ result ] = check_diff_state( old, new, data )
% %UNTITLED2 Summary of this function goes here
% %   Detailed explanation goes here
%     old = 
%     diff = sum((new - old).^2);
%     while
% 
% end
% 
old = record(:,1) ;
for i = 1:100
    new = record(:,i);
    diff = sum( (new-old).^2 );
    disp(diff);
    old = new;
    pause(0.5);
end
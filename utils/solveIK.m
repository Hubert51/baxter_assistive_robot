function [ my_qs ] = solveIK(robotArm, pos, ori, side)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
torlence = 0.005;
for i=1:8
    my_qs = robotArm.solveIKfast(pos, ori, side);
    if  isempty(my_qs)
        joint_diff = [2];
        pos = pos +torlence * (rand(3,1) - 0.5);
        if i >= 5
            ori = ori +torlence * (rand(4,1) - 0.5);
        end
    else
        break
    end
end

%         i = 1;
%         while ~isempty(find( abs(joint_diff) > 1.5, 1))
%             %pos = pos +torlence * (rand(3,1) - 0.5);
%             % ori = ori +torlence * (rand(4,1) - 0.5);
%             my_qs = robotArm.solveIKfast(pos, ori, side);
% 
%             if  isempty(my_qs)
%                 joint_diff = [1];
%             else
%                 joint_diff = my_qs - joint_pos;
%             end
% 
%             disp 'recalculate trajectory';
%             disp(i);
%             i = i+1;
% 
%             if i > 20
%                 my_qs = [];
%                 break;
%             end
%         end
%     end


% end


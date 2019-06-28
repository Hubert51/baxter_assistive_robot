function [ my_qs ] = solveIK(robotArm, robotPeripheries, pos, ori, side)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    torlence = 0.005;
    offset = 0;
    r= 0.36;
    deltaTheta = 10/180*pi;

    joint_pos = robotArm.joint_positions;
    if strcmp(side, 'right')
        joint_pos = joint_pos(8:14);
    end
    my_qs = robotArm.solveIKfast(pos, ori, side);
    if  isempty(my_qs)
        joint_diff = [1];
    else
        joint_diff = my_qs - joint_pos;
    end
    i = 1;
    while ~isempty(find( abs(joint_diff) > 0.7, 1))
        pos = pos +torlence * (rand(3,1) - 0.5);
        ori = ori +torlence * (rand(4,1) - 0.5);
        my_qs = robotArm.solveIKfast(pos, ori, side);

        if i > 50 || isempty(my_qs)
            % rotate five degree
%             base2right = robotPeripheries.lookUptransforms('/base', ...
%                             '/right_hand');
%             Hbase2right = quat2tform([base2right.quaternion(4); ...
%                 base2right.quaternion(1:3)]');
%             Hbase2right(1:3,4) = base2right.position;
% 
%             base2handle = Hbase2right * ...
%                 [axang2rotm([0 1 0 0.5 * deltaTheta]), ...
%                 [-r*(1-cos(deltaTheta)) 0 r*sin(deltaTheta)+offset]'; ...
%                 0 0 0 1];
%             % the position from homogenous transformation
%             % position_temp = base2handle(1:3,4);
%             ori = rotm2quat(base2handle(1:3,1:3))';
            base2right = robotPeripheries.lookUptransforms('/base', ...
                                '/right_hand');
            Rbase2right = quat2rotm([base2right.quaternion(4); ...
                base2right.quaternion(1:3)]');
            ori = rotm2quat(Rbase2right*axang2rotm([0 1 0 0.5*deltaTheta]))';
            my_qs = robotArm.solveIKfast(pos, ori, side);
        end
        
        joint_diff = my_qs - joint_pos;
        disp 'recalculate trajectory';
        disp(i);
        i = i+1;
        
        if i > 100
            my_qs = [];
            break;
        end
    end


end


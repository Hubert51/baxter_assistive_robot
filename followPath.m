function [  ] = followPath( robotArm, record )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    for i = 1:length(record(1,:))
        endeffector = record(:, i);
        robotArm.setJointCommand('left',endeffector(1:7));
        robotArm.setJointCommand('right',endeffector(8:14));
        pause(0.3);
    end


end


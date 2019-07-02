function [ ] = adjustPose( robotArm, side, axis )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    while ~prod(robotArm.getForces(side) < 8)
        force = robotArm.getForces(side);
        pos = robotArm.getPositions(side);
        pos = pos - 0.005*(sign(force) .* abs(force) > 8) .* axis;
        ori = robotArm.getOrientations(side);
        my_qs = solveIK(robotArm,  pos, ori, side);
        if ~isempty(my_qs)
            robotArm.setJointCommand(side, my_qs);
        end
    end


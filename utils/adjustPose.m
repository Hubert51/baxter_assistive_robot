function [ ] = adjustPose( robotArm, robotPeripheries, side, axis )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    side_camera = strcat( '/', side, '_hand_camera' );
    my_base2cam = robotPeripheries.lookUptransforms('base', side_camera);
    q2 = my_base2cam.quaternion;
    q2 = [q2(4); q2(1:3)]';
    R2 = quat2rotm(q2);
    % in the zero configuration, [0 0 1] is moving end_effector towards the
    % direction of gripper.
    p = R2 * [0; 0; -1];
    
    force = robotArm.getForces(side);
    while force(1) > 3
        force = robotArm.getForces(side);
        pos = robotArm.getPositions(side);
        pos = pos + 0.008*p;
        ori = robotArm.getOrientations(side);
        my_qs = solveIK(robotArm,  pos, ori, side);
        if ~isempty(my_qs)
            robotArm.setJointCommand(side, my_qs);
            pause(0.5)
        end
    end
    p = R2 * [0; 1; 0];
    while force(2) < 9
        force = robotArm.getForces(side);
        pos = robotArm.getPositions(side);
        pos = pos + 0.03*p;
        ori = robotArm.getOrientations(side);
        my_qs = solveIK(robotArm,  pos, ori, side);
        if ~isempty(my_qs)
            robotArm.setJointCommand(side, my_qs);
            pause(0.5)
        end
    end

    
    
    
    %pos = pos - 0.005*(sign(force) .* abs(force) > 8) .* axis;


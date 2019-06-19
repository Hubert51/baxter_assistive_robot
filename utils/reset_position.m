function [ ] = reset_position( side, robotArm )
    joint_angle = robotArm.joint_positions;
    if strcmp( side, 'right' )
        robotArm.setJointCommand('right', joint_angle(8:14));
    elseif strcmp( side, 'left' )
        robotArm.setJointCommand('left', joint_angle(1:7));
    end

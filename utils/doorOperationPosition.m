function [ output_args ] = doorOperationPosition( robotPeripheries, robotArm, side, dir )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% dir: -1 is open the door
%      1 close the door

% while True:
%     move the door
%     check the force 
%     if force is large:
%         rotate the gripper
%     else
%         contrinue


robotArm.setPositionModeSpeed(0.15); % slowly (unnecessary...)
fix_pos = robotArm.endeffector_positions;
fix_pos = fix_pos(4:6);
old_joint_pos = robotArm.joint_positions;
offset = 0;
my_hand =  strcat( '/', side, '_hand' );

for j = 1:11
    robotArm.setPositionModeSpeed(0.15); % slowly (unnecessary...)
    if j == 1
        robotArm.setPositionModeSpeed(0.09); % slowly (unnecessary...)
    end
    joint_pos = robotArm.getJointPositions(side);
    % 3X1 of position and 4*1 of quaternion
    base2right = robotPeripheries.lookUptransforms('/base', my_hand);
    Hbase2right = quat2tform([base2right.quaternion(4); ...
        base2right.quaternion(1:3)]');
    Hbase2right(1:3,4) = base2right.position;

    % original parameter
    base2handle = Hbase2right * ...
        [axang2rotm([0 1 0 deltaTheta]), ...
        [-r*(1-cos(deltaTheta)) 0 r*sin(deltaTheta)+offset]'; ...
        0 0 0 1];
    position_temp = fix_pos + [-r*(sin(deltaTheta*j)); -r*(1-cos(deltaTheta*j)); 0];

    % the position from homogenous transformation
    % position_temp = base2handle(1:3,4);
    orientation_temp = rotm2quat(base2handle(1:3,1:3))';

    % each part requires an IK 
    rightqs_temp = robotArm.solveIKfast(position_temp, ...
        orientation_temp, side);
    joint_diff = rightqs_temp - joint_pos;

    % this section try to make the opening door more smoothly.
    % if the joint angle changes largely, the code will
    % calculate another target position and recalculate the
    % inverse kinematics.
    temp_offset = 0;
    while ~isempty(find( abs(joint_diff) > 0.7, 1))
        leftqs_temp = robotArm.solveIKfast(position_temp, ...
        orientation_temp, 'left');
        position_temp = position + (rand(3,1)/500 - 0.001);
        joint_diff = leftqs_temp - joint_pos;
        disp 'recalculate trajectory';
    end
    % to give the offset back the system in next iteration.
    offset = -temp_offset;

    if ~isempty(rightqs_temp)
        robotArm.setJointCommand(side, rightqs_temp);
    end
    pause(0.3);
    while ~prod( abs(robotArm.joint_velocities) < 0.06); end
    % torque = robotArm.joint_torques;
    % real_pos = robotArm.endeffector_positions;
    % pos_diff = position_temp - real_pos(4:6)
    % torque = torque(8:14)
    % pause(0.3);

    reset_position(side, robotArm);
end
robotArm.setPositionModeSpeed(0.3);
function [ output_args ] = doorOperation( robotPeripheries, robotArm, dir )
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

side = 'right';
step_size = 0.05;
torlence = 0.005;
deltaTheta = 10/180*pi;
r = 0.36; % The radius of the fridge door
robotArm.setPositionModeSpeed(0.15); % slowly (unnecessary...)


while 1 
    joint_pos = robotArm.joint_positions;
    joint_pos = joint_pos(8:14);
    side_camera = strcat( '/', side, '_hand_camera' );
    my_base2cam = robotPeripheries.lookUptransforms('base', side_camera);
    q2 = my_base2cam.quaternion;
    q2 = [q2(4); q2(1:3)]';
    R2 = quat2rotm(q2);
    % in the zero configuration, [0 0 1] is moving end_effector towards the
    % direction of gripper.
    p = R2 * [0; 0; dir];
    p = p * step_size;

    pos = robotArm.endeffector_positions;
    pos = pos(4:6) + p;
    ori = robotArm.endeffector_orientations;
    ori = ori(5:8);
    
    eul = quat2eul(ori');
    eul(2) = 0;
    ori = eul2quat(eul)';
    
    my_qs = solveIK(robotArm,robotPeripheries, pos, ori, side);
    if ~isempty(my_qs)
        while ~prod(abs(robotArm.joint_velocities) < 0.05); end
        robotArm.setJointCommand(side, my_qs);
    end
    
    %% check the force 
    % if force > 8:
    %     try to rotate
    %     if rotate:
    %         break
    %     elseif not rotate:
    %         moveback and rotate.
    pause(1.5)
    wrenches = robotArm.endeffector_wrenches;
    wrenches = wrenches(10:12)
    % first move back to the previous state, and then rotate
    while ~isempty(find( abs(wrenches) > 8, 1)) || isempty(my_qs)
        % first rotate
        offset = 0;
        pos = robotArm.endeffector_positions;
        pos = pos(4:6);
%         base2right = robotPeripheries.lookUptransforms('/base', ...
%                             '/right_hand');
%         Hbase2right = quat2tform([base2right.quaternion(4); ...
%             base2right.quaternion(1:3)]');
%         Hbase2right(1:3,4) = base2right.position;
% 
%         base2handle = Hbase2right * ...
%             [axang2rotm([0 1 0 1.5*deltaTheta]), ...
%             [-r*(1-cos(deltaTheta)) 0 r*sin(deltaTheta)+offset]'; ...
%             0 0 0 1];
%         % the position from homogenous transformation
%         % position_temp = base2handle(1:3,4);
%         ori = rotm2quat(base2handle(1:3,1:3))';
        base2right = robotPeripheries.lookUptransforms('/base', ...
                            '/right_hand');
        Rbase2right = quat2rotm([base2right.quaternion(4); ...
            base2right.quaternion(1:3)]');
        
        rotate_sign = sign(wrenches);
        rotate_axis = rotate_sign .* (abs(wrenches) == max(abs(wrenches)));
        if abs(rotate_axis(3)) == 1 || abs(rotate_axis(1)) == 1
            disp 1
            a = 1 + 1;
        end
        if isempty(my_qs)
            ori = rotm2quat(Rbase2right*axang2rotm([0 1 0 1.5*deltaTheta]))';
        else
            ori = rotm2quat(Rbase2right*axang2rotm([rotate_axis' 1.5*deltaTheta]))';
        end
%         eul = quat2eul(ori');
%         eul(2) = 0;
%         ori = eul2quat(eul)';
        % each part requires an IK 
        my_qs = solveIK(robotArm, robotPeripheries, pos, ori, side);
        if ~isempty(my_qs)
            while ~prod(abs(robotArm.joint_velocities) < 0.05); end
            robotArm.setJointCommand('right', my_qs);
        else
            pos = robotArm.endeffector_positions;
            pos = pos(4:6) - 1.5 * p;
            my_qs = solveIK(robotArm,robotPeripheries, pos, ori, side);
            while ~prod(abs(robotArm.joint_velocities) < 0.05); end
            robotArm.setJointCommand(side, my_qs);
        end
        
        while ~prod(abs(robotArm.joint_velocities) < 0.05); end
        pause(1.5);
        wrenches = robotArm.endeffector_wrenches;
        wrenches = wrenches(10:12)
    end
    
end
    


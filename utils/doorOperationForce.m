function [ output_args ] = doorOperationForce( robotPeripheries, robotArm, dir )
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
totalTheta = 0;
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
    ori = robotArm.endeffector_orientations;
    ori = ori(5:8);
        wrenches = robotArm.endeffector_wrenches;
    wrenches = wrenches(10:12)
    eul = quat2eul(ori');
    eul(2) = 0;
    ori = eul2quat(eul)';
    
    i = 0;
    pos = robotArm.endeffector_positions;
    while i < 20
        my_pos = pos(4:6) + 0.5 * p + p*rand();
        my_qs = solveIK(robotArm, my_pos, ori, side);
        if ~isempty(my_qs)
            while ~prod(abs(robotArm.joint_velocities) < 0.05); end
            robotArm.setJointCommand(side, my_qs);
            break
        end
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
        
        while i < 20
            delta_angle = 1.5*deltaTheta + 2 * rand() * deltaTheta;
            ori = rotm2quat(Rbase2right*axang2rotm([0 1 0 delta_angle]))';
            my_qs = solveIK(robotArm, pos, ori, side);
            if ~isempty(my_qs)
                while ~prod(abs(robotArm.joint_velocities) < 0.05); end
                pause(0.5)
                robotArm.setJointCommand('right', my_qs);
                totalTheta = totalTheta + delta_angle;
                break
%             else
%                 pos = robotArm.endeffector_positions;
%                 pos = pos(4:6) - 1.5 * p * rand();
%                 my_qs = solveIK(robotArm, pos, ori, side);
%                 while ~prod(abs(robotArm.joint_velocities) < 0.05); end
%                 pause(0.5)
%                 robotArm.setJointCommand(side, my_qs);
%                 break
            end
        end
        if totalTheta > 100*pi/180
            break
        end
        while ~prod(abs(robotArm.joint_velocities) < 0.05); end
        pause(1.5);
        wrenches = robotArm.endeffector_wrenches;
        wrenches = wrenches(10:12);
    end
    if totalTheta > 100*pi/180
        break
    end
    
end
    


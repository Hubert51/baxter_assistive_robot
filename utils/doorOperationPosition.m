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

if nargin > 3
  dir = dir;
else
  dir = -1;
end
deltaTheta = 10/180*pi;
r = 0.375; % The radius of the fridge door
robotArm.setPositionModeSpeed(0.15); % slowly (unnecessary...)
fix_pos = robotArm.endeffector_positions;
fix_pos = fix_pos(4:6);
old_joint_pos = robotArm.joint_positions;
offset = 0;
my_hand =  strcat( '/', side, '_hand' );

for j = 1:9
    robotArm.setPositionModeSpeed(0.15); % slowly (unnecessary...)
    if j == 1
        robotArm.setPositionModeSpeed(0.09); % slowly (unnecessary...)
    end
    joint_pos = robotArm.getJointPositions(side);
    
    % the orientation only changes when in first 9 iteration which is 
    % 90 degree
    if j < 10
        % 3X1 of position and 4*1 of quaternion
        % base2right = robotPeripheries.lookUptransforms('/base', my_hand);
        Rbase2right = quat2rotm(robotArm.getOrientations(side)');

        % original parameter
        if dir == -1
            base2handle = Rbase2right * axang2rotm([0 1 0 deltaTheta]);
            position_temp = fix_pos + [-r*(sin(deltaTheta*j)); -r*(1-cos(deltaTheta*j)); 0];
        elseif dir == 1
            base2handle = Rbase2right * axang2rotm([0 -1 0 deltaTheta]);
            position_temp = fix_pos + [r*(1-cos(deltaTheta*(j))); r*(sin(deltaTheta*(j) )); 0];
        end


        % the position from homogenous transformation
        % position_temp = base2handle(1:3,4);
        orientation_temp = rotm2quat(base2handle(1:3,1:3))';
        
    else
        R2 = quat2rotm(robotArm.getOrientations(side)');
        % in the zero configuration, [0 0 1] is moving end_effector towards the
        % direction of gripper.
        p = R2 * [0; 0; -1];
        position_temp = robotArm.getPositions(side) + 0.5 * p;
    end

    % each part requires an IK 
    rightqs_temp = robotArm.solveIKfast(position_temp, ...
        orientation_temp, side);

    % this section try to make the opening door more smoothly.
    % if the joint angle changes largely, the code will
    % calculate another target position and recalculate the
    % inverse kinematics.
    count = 0;
    while isempty(rightqs_temp) || ~isempty(find( abs(rightqs_temp - joint_pos) > 0.7, 1))
        rightqs_temp = robotArm.solveIKfast(position_temp, ...
        orientation_temp, 'right');
        if ~isempty(rightqs_temp)
            robotArm.setJointCommand(side, rightqs_temp);
            pause(0.3)
            while ~prod( abs(robotArm.joint_velocities) < 0.06); 
                if  ~prod(abs(robotArm.getForces('right')) < 13)
                    robotArm.setJointCommand(side, joint_pos);
                    rightqs_temp = [];
                    break
                end
            end
            if (abs(robotArm.getJointPositions(side) -rightqs_temp )<=0.3)
                break
            end
        end
        position_temp = position_temp + (rand(3,1)/50 - 0.01);
        disp 'recalculate trajectory';
        count = count + 1;
        if count == 20
            break
        end
    end
    % to give the offset back the system in next iteration.

    if ~isempty(rightqs_temp)
        robotArm.setJointCommand(side, rightqs_temp);
    end
    pause(0.3);
    while ~prod( abs(robotArm.joint_velocities) < 0.06); end
    % torque = robotArm.joint_torques;
    % real_pos = robotArm.endeffector_positions;
    % pos_diff = position_temp - real_pos(4:6)
    % torque = torque(8:14)
    pause(0.3);
    force = robotArm.getForces(side);
    if force > 10
        break;
    end    
    reset_position(side, robotArm);

end

if dir == 1
    force = robotArm.getForces(side);
    i = 0;
    while i == 0 || force(1) < 9
        i = 1;
        position_temp = robotArm.getPositions(side) + rand() * [0.05; 0; 0];
        rightqs_temp = robotArm.solveIKfast(position_temp, ...
            orientation_temp, side);
        if ~isempty(rightqs_temp)
            robotArm.setJointCommand(side, rightqs_temp);
            pause(0.3)
            while ~prod( abs(robotArm.joint_velocities) < 0.06); end
            pause(0.3)
        end
        force = robotArm.getForces(side);   
    end
end
reset_position(side, robotArm);
robotArm.setPositionModeSpeed(0.3);
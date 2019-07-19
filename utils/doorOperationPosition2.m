function [ output_args ] = doorOperationPosition2(robotArm, radius, side, dir )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% dir: -1 is close the door
%      1 open the door

% while True:
%     move the door
%     check the force 
%     if force is large:
%         rotate the gripper
%     else
%         contrinue
k = {'left', 'l', 'right', 'r'};
v = {-1, -1, 1, 1};
side_flag = containers.Map(k, v);

if nargin > 3
  dir = dir;
else
  dir = 1;
end
deltaTheta = 10/180*pi * side_flag(side);
r = radius; % The radius of the fridge door
robotArm.setPositionModeSpeed(0.15); % slowly (unnecessary...)
fix_pos = robotArm.getPositions(side);
old_joint_pos = robotArm.joint_positions;
offset = 0;
my_hand =  strcat( '/', side, '_hand' );
k = {'left', 'l', 'right', 'r'};
v = {-1, -1, 1, 1};
side_flag = containers.Map(k, v);

robotArm.setPositionModeSpeed(0.15); % slowly (unnecessary...)
% if dir == -1
%     q2 = robotArm.getOrientations(side);
%     R2 = quat2rotm(q2');
%     % in the zero configuration, [0 0 1] is moving end_effector towards the
%     % direction of gripper.
%     p = R2 * [0; 0; -1] * 0.03 * dir;
%     force = robotArm.getForces(side);
%     i = 0;
%     while i < 5  && ( prod(abs(force) < 8) )
%         i = i + 1;
%         position_temp = robotArm.getPositions(side) + p;
%         rightqs_temp = robotArm.solveIKfast(position_temp, ...
%             robotArm.getOrientations(side), side);
%         if ~isempty(rightqs_temp)
%             robotArm.setJointCommand(side, rightqs_temp);
%             pause(0.3)
%             while ~prod( abs(robotArm.getJointVelocities(side)) < 0.06); end
%             pause(0.3)
%         end
%         force = robotArm.getForces(side);   
%     end
% end


fix_pos = robotArm.getPositions(side);
j = 0;

points = cell(10,1);
points{1} = struct;
points{1}.pos = robotArm.getPositions(side);
points{1}.ori = robotArm.getOrientations(side);
robotArm.setPositionModeSpeed(0.09);

Rbase2right = quat2rotm(robotArm.getOrientations(side)');
while j < 9
    j = j + 1;
    if j == 1
        robotArm.setPositionModeSpeed(0.09); % slowly (unnecessary...)
    end
    joint_pos = robotArm.getJointPositions(side);
    
    % the orientation only changes when in first 9 iteration which is 
    % 90 degree
    if j < 10
        % 3X1 of position and 4*1 of quaternion
        % base2right = robotPeripheries.lookUptransforms('/base', my_hand);

        % original parameter
        if dir == 1
            base2handle = Rbase2right * axang2rotm([0 dir 0 j * deltaTheta]);
            p = [-r*(sin(deltaTheta*j)); -r*(1-cos(deltaTheta*j)); 0];
            position_temp = fix_pos + p*side_flag(side);
        elseif dir == -1
            base2handle = Rbase2right * axang2rotm([0 dir 0 j*deltaTheta]);
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
        position_temp = robotArm.getPositions(side) + 0.2 * p;
    end

    % each part requires an IK 
    rightqs_temp = robotArm.solveIKfast(position_temp, ...
        orientation_temp, side);

    % this section try to make the opening door more smoothly.
    % if the joint angle changes largely, the code will
    % calculate another target position and recalculate the
    % inverse kinematics.
    count = 0;
%     while isempty(rightqs_temp) || ~isempty(find( abs(rightqs_temp - joint_pos) > 1, 1))
%         % since here we need to make sure the arm will stop and go back if
%         % if large force on the arm.
%         robotArm.setPositionModeSpeed(0.09);
%         rightqs_temp = robotArm.solveIKfast(position_temp, ...
%         orientation_temp, side);
%         if ~isempty(rightqs_temp) && ~isempty(find( abs(rightqs_temp - joint_pos) > 1, 1))
%             if ~isempty(find( abs(rightqs_temp - joint_pos) > 0.7, 1))
%                 disp(abs(rightqs_temp - joint_pos))
%             end
%             robotArm.setJointCommand(side, rightqs_temp);
%             pause(0.3)
%             robotArm.getForces(side)
%             while ~prod( abs(robotArm.getJointVelocities(side)) < 0.06); 
%                 if  ~prod(abs(robotArm.getForces(side)) < 11)
%                     robotArm.setJointCommand(side, joint_pos);
%                     rightqs_temp = [];
%                     break
%                 end
%             end
%             if ~isempty(rightqs_temp) && prod(abs(robotArm.getJointPositions(side) -rightqs_temp )<=0.3)
%                 break
%             end
%         end
%         position_temp = position_temp + (rand(3,1)/50 - 0.01);
%         disp 'recalculate trajectory';
%         count = count + 1
%         if count >= 15 
%             orientation_temp = robotArm.getOrientations(side);
%             if count == 15
%                 j = j - 1;
%             end
%         elseif count == 30
%             input('Need help!')
%         end
%     end
%     robotArm.setPositionModeSpeed(0.09);
    % to give the offset back the system in next iteration.
    points{1+j} = points{1};
    points{1+j}.pos = position_temp;
    points{1+j}.ori = orientation_temp;
%     if ~isempty(find( abs(rightqs_temp - joint_pos) > 1, 1))
%         disp(abs(rightqs_temp - joint_pos))
%     end
%     if ~isempty(rightqs_temp)
%         robotArm.setJointCommand(side, rightqs_temp);
%     end
%     pause(0.15);
%     while ~prod( abs(robotArm.getJointVelocities(side)) < 0.03); 
%         disp(j)
%     end
    % torque = robotArm.joint_torques;
    % real_pos = robotArm.endeffector_positions;
    % pos_diff = position_temp - real_pos(4:6)
    % torque = torque(8:14)
%     pause(0.1);
%     robotArm.getForces(side)
%     if ~prod(abs(robotArm.getForces(side)) < 13) && j > 5
%         reset_position(side, robotArm);
%         break;
%     end    
    % reset_position(side, robotArm);

end
robotArm.setPositionModeSpeed(0.05);
frac = robotArm.cartesianPathTraj(side, points);
pause(1)
if frac == 1
    robotArm.execute();
end
pause(1)
% robotArm.setPositionModeSpeed(0.1);

% for i=2:9
%     robotArm.poseTargetTraj(side, points{i}.pos, points{i}.ori);
%     robotArm.execute()
%     pause(0.1)
%     while ~prod( abs(robotArm.getJointVelocities(side)) < 0.05); end
% end

% q2 = robotArm.getOrientations(side);
% R2 = quat2rotm(q2');
% % in the zero configuration, [0 0 1] is moving end_effector towards the
% % direction of gripper.
% p = R2 * [0; 0; -1] * 0.03 * dir;
% % [0 0 1] is the axis of force of the direction of the gripper
% filter = R2 * [0; 0; 1];
% force = filter .* robotArm.getForces(side);
% i = 0;
% while i < 5 && ( prod(abs(force) < 9) )
%     i = i+1;
%     position_temp = robotArm.getPositions(side) + p;
%     rightqs_temp = robotArm.solveIKfast(position_temp, ...
%         orientation_temp, side);
%     if ~isempty(rightqs_temp)
%         robotArm.setJointCommand(side, rightqs_temp);
%         pause(0.3)
%         while ~prod( abs(robotArm.getJointVelocities(side)) < 0.06); end
%         pause(0.3)
%     end
%     force = filter .* robotArm.getForces(side);   
% end
% 
% reset_position(side, robotArm);
% robotArm.setPositionModeSpeed(0.15);

side='right';
Rbase2right = quat2rotm(robotArm.getOrientations(side)');
Rbase2right * [0 0 -1]'
base2handle = Rbase2right * axang2rotm([0 1 0 deltaTheta]);
orientation_temp = rotm2quat(base2handle(1:3,1:3));

side_camera = strcat( '/', side, '_hand_camera' );
my_base2cam = robotPeripheries.lookUptransforms('base', side_camera);
q2 = my_base2cam.quaternion;
% q2 = [q2(4); q2(1:3)]';
R2 = quat2rotm(q2');
R2 * [0 0 -1]'
base2handle = R2 * axang2rotm([1 0 0 deltaTheta]);
orientation_temp = rotm2quat(base2handle(1:3,1:3));
% in the zero configuration, [0 0 1] is moving end_effector towards the
% direction of gripper.


base2right = robotPeripheries.lookUptransforms('/base', 'right_hand');
Rbase2right = quat2rotm([base2right.quaternion(4); ...
    base2right.quaternion(1:3)]');
Rbase2right * [0 0 -1]'

% original parameter
base2handle = Rbase2right * axang2rotm([0 1 0 deltaTheta]);
% position_temp = fix_pos + [-r*(sin(deltaTheta*j)); -r*(1-cos(deltaTheta*j)); 0];

% the position from homogenous transformation
% position_temp = base2handle(1:3,4);
orientation_temp = rotm2quat(base2handle(1:3,1:3))




% 
% function [ output_args ] = draft2( robotPeripheries, robotArm, dir )
% %UNTITLED2 Summary of this function goes here
% %   Detailed explanation goes here
% 
% % dir: -1 is open the door
% %      1 close the door
% 
% % while True:
% %     move the door
% %     check the force 
% %     if force is large:
% %         rotate the gripper
% %     else
% %         contrinue
% 
% side = 'right';
% step_size = 0.01;
% torlence = 0.005;
% deltaTheta = 10/180*pi;
% r = 0.36; % The radius of the fridge door
% robotArm.setPositionModeSpeed(0.15); % slowly (unnecessary...)
% 
% positive = 0;
% negative = 0;
% while 1 
%     disp( [ positive, negative ]);
%     joint_pos = robotArm.joint_positions;
%     joint_pos = joint_pos(8:14);
%     side_camera = strcat( '/', side, '_hand_camera' );
%     my_base2cam = robotPeripheries.lookUptransforms('base', side_camera);
%     q2 = my_base2cam.quaternion;
%     q2 = [q2(4); q2(1:3)]';
%     R2 = quat2rotm(q2);
%     % in the zero configuration, [0 0 1] is moving end_effector towards the
%     % direction of gripper.
%     p = R2 * [0; 0; dir];
%     p = p * step_size;
% 
%     pos = robotArm.endeffector_positions;
%     pos = pos(4:6) + p;
%     ori = robotArm.endeffector_orientations;
%     ori = ori(5:8);
%     my_qs = robotArm.solveIKfast(pos, ori, side);
%     if ~isempty(my_qs)
%         while ~prod(abs(robotArm.joint_velocities) < 0.05); end
%         robotArm.setJointCommand(side, my_qs);
%         positive = positive + 1;
%     else
%         negative = negative + 1;
%     end    
% end
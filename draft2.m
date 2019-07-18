points = cell(1183,1);
i = 1;
for key = RoadMap.keys
    part_map = RoadMap(char(key));
    point = struct;
    point.pos = part_map.pos;
    point.ori = part_map.ori;
    points{i} = point ;
    i = i+1;
end
% point = RoadMap('init_ptr');
% robotArm.setJointCommand('right', point.qs);
% pause(1);
% while ~prod(robotArm.joint_velocities < 0.03); end
% pause(1);
rightCamera = RobotRaconteur.Connect('tcp://localhost:4567/BaxterCameraServer/right_hand_camera');
robotArm = RobotRaconteur.Connect('tcp://localhost:2345/BaxterJointServer/Baxter');
arTagposes = rightCamera.ARtag_Detection();
index_f = find(arTagposes.ids == 5);
ori1 = robotArm.getOrientations('r');
H1 = quat2tform(ori1');
H1(1:3,4) = robotArm.getPositions('r');
right2tag = reshape(arTagposes.tmats((index_f-1)*16+1: index_f*16), ...
        4, 4);
R1 = axang2tform([1 0 0 0.5*pi]) * axang2tform([0 1 0 -0.5*pi]);
right2tag = R1 * right2tag;
ori1 = rotm2quat(right2tag(1:3,1:3))';
rightqs = robotArm.solveIKfast(robotArm.getPositions('r'), ori1, 'right');
robotArm.setJointCommand('right', rightqs);


right2tag(1:3,4) = - right2tag(1:3,4);
base2tag = H1 * right2tag;
position = base2tag(1:3,4) - [0.1 0 0]';
orientation = rotm2quat(base2tag(1:3,1:3))';
rightqs = robotArm.solveIKfast(position, ori1, 'right');
robotArm.setJointCommand('right', rightqs);

p1 = [0.0996; -0.1164; 0.3686];
bias = axang2rotm([1 0 0 0.5*pi]) * axang2rotm([0 1 0 -0.5*pi]);



Pose1.pos = robotArm.getPositions('r');
Pose1.ori = robotArm.getOrientations('r');

Pose2.pos = Pose1.pos + [0.1;0; -0.15];
Pose2.ori = Pose1.ori;
Pose = {Pose1; Pose2}';

ori_final = robotArm.getOrientations('l')
ori_init = [ -0.0027; 0.7184; 0.0193; 0.6953 ];
RR = quat2rotm(ori_init') * rot([-0 -0  -1], 50*pi/180) ;
rotm2quat(RR)

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
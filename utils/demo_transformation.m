% demo: transformation with tag
% use tag to determine the orientation of the gripper and position offset
% of the gripper. Combined with current position, we can let the gripper to
% the desired pose.

%% reset the position
% point = RoadMap('init_ptr');
% robotArm.setJointCommand('right', point.qs);
% pause(1);
% while ~prod(robotArm.joint_velocities < 0.03); end
% pause(1);

%%
rightCamera = RobotRaconteur.Connect('tcp://localhost:4567/BaxterCameraServer/right_hand_camera');
robotArm = RobotRaconteur.Connect('tcp://localhost:2345/BaxterJointServer/Baxter');
arTagposes = rightCamera.ARtag_Detection();
index_f = find(arTagposes.ids == 5);
right2tag = reshape(arTagposes.tmats((index_f-1)*16+1: index_f*16), 4, 4);

% transformation matrix for the coordinates
R1 = axang2tform([1 0 0 0.5*pi]) * axang2tform([0 1 0 -0.5*pi]);
right2tag = R1 * right2tag;
% the follow calculation is the orientation of current gripper
right2tag(1:3, 1:3) = axang2rotm([1 0 0 0.5*pi]) * right2tag(1:3, 1:3);
% H1(1:3, 1:3) = eye(3);
base2tag(1:3, 4) = -right2tag(1:3, 4) + robotArm.getPositions('r');

ori1 = rotm2quat(base2tag(1:3,1:3))';
pos1 = base2tag(1:3, 4) - [0.15; 0; 0];
rightqs = robotArm.solveIKfast(pos1, ori1, 'right');
robotArm.setJointCommand('right', rightqs);
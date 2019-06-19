% connecting to robotRR bridges
robotArm = RobotRaconteur.Connect('tcp://localhost:2345/BaxterJointServer/Baxter');
% rightCamera = RobotRaconteur.Connect('tcp://localhost:4567/BaxterCameraServer/right_hand_camera');
leftCamera = RobotRaconteur.Connect('tcp://localhost:9087/BaxterCameraServer/left_hand_camera');
robotPeripheries = RobotRaconteur.Connect('tcp://localhost:6708/BaxterPeripheralServer/BaxterPeripherals');

% set the mode=1 if using moveit module to move the arm.
% robotArm.setControlMode(uint8(1));
% robotArm.moveitSetJointCommand('both_arm', [1.0; 2.; 3.; 4.;5]);

% waiting for the initialization of the cameras
%rightCamera.openCamera();
leftCamera.openCamera();
pause(3.5); 

load('/home/cats/Douglas Robots/cameraparams_right.mat');
load('/home/cats/Douglas Robots/cameraparams_left.mat');
%rightCamera.setCameraIntrinsics(cameraparams_right);
leftCamera.setCameraIntrinsics(cameraparams_lefthand);

% Make sure the marker size is correct
%rightCamera.setMarkerSize(0.055);
leftCamera.setMarkerSize(0.055);
pause(1);

% robotArm.setControlMode(uint8(1));
% robotPeripheries.calibrateGripper('l');
% robotPeripheries.openGripper('l');
% 
% robotPeripheries.calibrateGripper('r');
% robotPeripheries.openGripper('r');    

% pause(1);

% set joint angle control mode 
robotArm.setControlMode(uint8(0));
load('/home/cats/Douglas Robots/left_init.mat');
load('/home/cats/Douglas Robots/right_init.mat');

% initial position affects the result of IK during the process a lot...
% robotArm.setJointCommand('left', left);
% robotArm.setJointCommand('right', right);
r_m = 0.295; % radius of the microwave door
%r_m = 0.34;
% data for open the microwave
deltaTheta = 10/180*pi;
robotArm.setPositionModeSpeed(0.3); 

headPan = (4-4)*pi/12;
base2headcam = robotPeripheries.lookUptransforms('/base', ...
    '/head_camera');
Hbase2headcam = quat2tform([base2headcam.quaternion(4); ...
    base2headcam.quaternion(1:3)]');
Hbase2headcam(1:3,4) = base2headcam.position;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% 4: move left hand to detect the microwave door %%%%%%%%%%%%%
% place left hand in front of the head camera
moveforward_l = [axang2rotm([1 0 0 -pi/16]), ...
    [-0.02 -0.1 0.6+0.8/pi*abs(headPan)]';...
    0 0 0 1]; % offset the looking down angle of the head pan 

Hbase2headside_l = ... 
    Hbase2headcam*moveforward_l;
position = Hbase2headside_l(1:3,4);
orientation = rotm2quat(Hbase2headside_l(1:3,1:3))';
leftqs_l = robotArm.solveIKfast(position, orientation, 'left');
robotArm.setJointCommand('left', leftqs_l);

pause(0.5);
while ~prod(robotArm.joint_velocities < 0.01); end
pause(2);

% detect the microwave tag
arTagposes_l = leftCamera.ARtag_Detection();
if isempty(arTagposes_l)
    system('spd-say "no tag detected"');
    break
end

index_m = find(arTagposes_l.ids == 2); % door tag
if isempty(index_m)
    system('spd-say "nothing detected"');
    break
end

% get base to left hand camera transform
base2leftcam = robotPeripheries.lookUptransforms('/base', ...
    '/left_hand_camera');
Hbase2leftcam = quat2tform([base2leftcam.quaternion(4); ...
    base2leftcam.quaternion(1:3)]');
Hbase2leftcam(1:3,4) = base2leftcam.position;

% receive the transform from left hand to tag 
left2tag = reshape( ...
    arTagposes_l.tmats((index_m-1)*16+1: index_m*16), 4, 4);

% the place to put the vacuum suction has an offset to the tag 
bias_l = axang2tform([0 1 0 pi]);
bias_l(1:3,4) = [0.09 0 0]';
base2door = Hbase2leftcam * left2tag * bias_l;
position = base2door(1:3,4);
orientation = rotm2quat(base2door(1:3,1:3))';

% solve IK for joint angles that move the gripper to the place
leftqs_m = robotArm.solveIKfast(position, orientation, 'left');
if ~isempty(leftqs_m)
    robotArm.setJointCommand('left', leftqs_m);
end
pause(1);
robotPeripheries.closeGripper('l');
while ~prod(robotArm.joint_velocities < 0.03); end
pause(1);
robotPeripheries.closeGripper('l');
pause(1);

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%% 5: open the microwave oven door %%%%%%%%%%%%%%%%%%%%%
% % the process is divided into 12 parts
% for j = 1:12
%     arTagposes_l = leftCamera.ARtag_Detection();
%     if isempty(arTagposes_l)
%         system('spd-say "no tag detected"');
%         continue;
%     end
% 
%     index_m = find(arTagposes_l.ids == 2); % door tag
%     if isempty(index_m)
%         system('spd-say "nothing detected"');
%         continue;
%     end
% 
%     % get base to left hand camera transform
%     base2leftcam = robotPeripheries.lookUptransforms('/base', ...
%         '/left_hand_camera');
%     Hbase2leftcam = quat2tform([base2leftcam.quaternion(4); ...
%         base2leftcam.quaternion(1:3)]');
%     Hbase2leftcam(1:3,4) = base2leftcam.position;
% 
%     % receive the transform from left hand to tag 
%     left2tag = reshape( ...
%         arTagposes_l.tmats((index_m-1)*16+1: index_m*16), 4, 4);
%     bias_l = axang2tform([0 1 0 pi]);
%     bias_l(1:3,4) = [0.087 0 0.12]';
%     base2door_temp = Hbase2leftcam * left2tag * bias_l * ...
%         axang2tform([0 1 0 deltaAlpha]);
%     position = base2door_temp(1:3,4);
%     orientation = rotm2quat(base2door_temp(1:3,1:3))';
% 
%     % solve IK for joint angles that move the gripper
%     leftqs_temp = robotArm.solveIKfast(position, ...
%         orientation, 'left');
%     if ~isempty(leftqs_temp)
%         robotArm.setJointCommand('left', leftqs_temp);
%     end
%     pause(0.2);
%     while ~prod(robotArm.joint_velocities < 0.06); end
%     pause(0.3);
% end

%% the open fridge door algo
robotArm.setPositionModeSpeed(0.15); % slowly (unnecessary...)

fix_pos = robotArm.endeffector_positions;
fix_pos = fix_pos(1:3);
for j = 1:9
    joint_pos = robotArm.joint_positions;
    joint_pos = joint_pos(1:7);
    % 3X1 of position and 4*1 of quaternion
    base2left = robotPeripheries.lookUptransforms('/base', ...
        '/left_hand');
    Hbase2left = quat2tform([base2left.quaternion(4); ...
        base2left.quaternion(1:3)]');

    % this transformation matrix does not include position change, only 
    % includes orientation change 
    base2handle = Hbase2left * ...
        [axang2rotm([0 1 0 -deltaTheta]), ...
        [0 0 0]'; ...
        0 0 0 1];
    
    orientation_temp = rotm2quat(base2handle(1:3,1:3))';
    % here is position changing.
    position_temp = fix_pos + [-r_m*(sin(deltaTheta*j)); r_m*(1-cos(deltaTheta*j)); 0];
    % each part requires an IK 
    leftqs_temp = robotArm.solveIKfast(position_temp, ...
        orientation_temp, 'left');
    joint_diff = leftqs_temp - joint_pos;

    % this section try to make the opening door more smoothly.
    % if the joint angle changes largely, the code will
    % calculate another target position and recalculate the
    % inverse kinematics.
    temp_offset = 0;
    while ~isempty(find( abs(joint_diff) > 0.7, 1))
        leftqs_temp = robotArm.solveIKfast(position_temp, ...
        orientation_temp, 'left');
        joint_diff = leftqs_temp - joint_pos;
        disp 'recalculate trajectory';
    end

    if ~isempty(leftqs_temp)
        robotArm.setJointCommand('left', leftqs_temp);
        disp j;
    end
    pause(0.3);
    % to wait for every joint move.
    while ~prod( abs(robotArm.joint_velocities) < 0.03); 
        robotArm.joint_velocities;
    end
    pause(1);
    reset_position('left', robotArm);
end
robotArm.setPositionModeSpeed(0.3);
            
%% get food from fridge(part1)            
robotPeripheries.openGripper('l');
pause(1);
% move the lefthand backward and downward to avoid collision
% into the microwave oven door
base2left = robotPeripheries.lookUptransforms('/base', ...
    '/left_hand');
Hbase2left = quat2tform([base2left.quaternion(4); ...
    base2left.quaternion(1:3)]');
Hbase2left(1:3,4) = base2left.position;
Hbase2idle = Hbase2left * [axang2rotm([0 1 0 pi/2]), ...
    [0 -0.2 -0.2]'; 0, 0, 0, 1];
position = Hbase2idle(1:3,4);
orientation = rotm2quat(Hbase2idle(1:3,1:3))';
leftqs_b = robotArm.solveIKfast(position, orientation, 'left');
if ~isempty(leftqs_b)
    robotArm.setJointCommand('left', leftqs_b);
end
pause(0.5);
while ~prod(robotArm.joint_velocities < 0.05); end
pause(1.5);
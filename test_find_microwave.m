    
% open the doors and transfer the food container
    % The following script makes the baxter on wheel open the fridge door
    % and take out a food container and put it into the microwave oven when
    % it is at the approximately-correct position
    %


clear
rosshutdown
rosinit('011303P0004.local','NodeHost','192.168.1.134');


% connecting to robotRR bridges
robotArm = RobotRaconteur.Connect('tcp://localhost:2345/BaxterJointServer/Baxter');
rightCamera = RobotRaconteur.Connect('tcp://localhost:4567/BaxterCameraServer/right_hand_camera');
leftCamera = RobotRaconteur.Connect('tcp://localhost:9087/BaxterCameraServer/left_hand_camera');
robotPeripheries = RobotRaconteur.Connect('tcp://localhost:6708/BaxterPeripheralServer/BaxterPeripherals');

% waiting for the initialization of the cameras
rightCamera.openCamera();
leftCamera.openCamera();
pause(3.5); 

load('/home/cats/Douglas Robots/cameraparams_right.mat');
load('/home/cats/Douglas Robots/cameraparams_left.mat');
rightCamera.setCameraIntrinsics(cameraparams_right);
leftCamera.setCameraIntrinsics(cameraparams_lefthand);

% Make sure the marker size is correct
rightCamera.setMarkerSize(0.055);
leftCamera.setMarkerSize(0.055);

pause(1);
% Sometimes the following line does not work...
robotPeripheries.calibrateGripper('r');
pause(1);

% set joint angle control mode 
robotArm.setControlMode(uint8(0));
load('/home/cats/Douglas Robots/left_init.mat');
load('/home/cats/Douglas Robots/right_init.mat');

% initial position affects the result of IK during the process a lot...
robotArm.setJointCommand('left', left);
robotArm.setJointCommand('right', right);

% waiting for movements complete
pause(1);
while ~prod(robotArm.joint_velocities < 0.1); end
pause(0.5);

% some data
deltaTheta = 10/180*pi;
    % The process of opening the fridge door is divided into nine parts.
    % Every part moves the door open 10 degrees.
deltaAlpha = -pi/2/12;
robotArm.setPositionModeSpeed(0.3);
r = 0.3; % The radius of the fridge door

for i = 1:7
    
    % looking for the tag
    headPan = (i-4)*pi/12;
    robotPeripheries.panHead(headPan);
    pause(1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% 1: move the right hand to look for tags %%%%%%%%%%%%%%%%%
    % get base to head camera transform
    base2headcam = robotPeripheries.lookUptransforms('/base', ...
        '/head_camera');
    Hbase2headcam = quat2tform([base2headcam.quaternion(4); ...
        base2headcam.quaternion(1:3)]');
    Hbase2headcam(1:3,4) = base2headcam.position;
    
    % place right hand below the head camera
    moveforward = [axang2rotm([1 0 0 -pi/16]), ...
        [0 -0.45 0.675+0.8/pi*abs(headPan)]';...
        0 0 0 1]; % pi/16 offsets the looking down angle of the head pan 
    
    Hbase2headside = ... 
        Hbase2headcam*moveforward;
    position = Hbase2headside(1:3,4);
    orientation = rotm2quat(Hbase2headside(1:3,1:3))';
    rightqs = robotArm.solveIKfast(position, orientation, 'right');
    robotArm.setJointCommand('right', rightqs);
    
    pause(0.5);


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
    bias_l(1:3,4) = [0.09 0 0.08]';
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
    
end

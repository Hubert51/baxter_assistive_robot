function [ output_args ] = demoSearchFridge( arTagposes )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    robotArm = RobotRaconteur.Connect('tcp://localhost:2345/BaxterJointServer/Baxter');
    rightCamera = RobotRaconteur.Connect('tcp://localhost:4567/BaxterCameraServer/right_hand_camera');
    leftCamera = RobotRaconteur.Connect('tcp://localhost:9087/BaxterCameraServer/left_hand_camera');
    robotPeripheries = RobotRaconteur.Connect('tcp://localhost:6708/BaxterPeripheralServer/BaxterPeripherals');
    load('Hubert1.mat')
    r = 0.38;
    robotArm.setPositionModeSpeed(0.12);

    index_f = find(arTagposes.ids == 0); % upper fridge handle
    if isempty(index_f)
        disp('need help')
        robotArm.setControlMode(uint8(1));
        input('please adjust the pose')
        robotArm.setControlMode(uint8(0));
    end
    
    arTagposes = rightCamera.ARtag_Detection();
    index_f = find(arTagposes.ids == 0); % upper fridge handle

    base2rightcam = robotPeripheries.lookUptransforms('/base', ...
        '/right_hand_camera');
    Hbase2rightcam = quat2tform([base2rightcam.quaternion(4); ...
        base2rightcam.quaternion(1:3)]');
    Hbase2rightcam(1:3,4) = base2rightcam.position;
    
    % receive the transform from right hand to tag 
    right2tag = reshape(arTagposes.tmats((index_f-1)*16+1: index_f*16), ...
        4, 4);
    
    % the position we are gonna put the hand in front of the handle has a 
    % bias from the tag. 
    bias = axang2tform([1 0 0 -pi]);
    bias(1:3, 4) = [-0.1 -0. -0.045]';
    bias = bias*axang2tform([0 0 1 pi])*axang2tform([1 0 0 -pi/36]);
    base2tag = Hbase2rightcam * right2tag * bias;
   
    position = base2tag(1:3,4);

    
    % the position of the area at the front of the fridge.
    % for left hand
    % front_fridge_pos = Hbase2headside(1:3,4) + [0.115; 0.06; 0];
    orientation = rotm2quat(base2tag(1:3,1:3))';
    robotArm.setControlMode(uint8(0));
    rightqs = robotArm.solveIKfast(position, orientation, 'right');
    robotArm.setJointCommand('right', rightqs);
    pause(1)
    while ~prod(abs(robotArm.joint_velocities) < 0.05); end
    pause(1);
    robotPeripheries.closeGripper('r');
    adjustPose( robotArm, robotPeripheries, 'right', [1; 1; 1] );
    doorOperationPosition(robotArm, r, 'right')
    
    % move the left arm
    leftqs_fridge = robotArm.solveIKfast(fridge_door_pos, fridge_door_ori2, 'left');
    if ~isempty(leftqs_fridge)
        robotArm.setJointCommand('left', leftqs_fridge2);
    end
    pause(0.5);
    while ~prod(abs(robotArm.joint_velocities) < 0.03); end
    pause(3);
    
    pos_down = fridge_door_pos + [0; 0; -0.3];
    leftqs_down = robotArm.solveIKfast(pos_down, fridge_door_ori2, 'left');
    if ~isempty(leftqs_down)
        robotArm.setJointCommand('left', leftqs_down);
        pause(0.5);
        while ~prod(abs(robotArm.joint_velocities) < 0.03); end
        pause(3);
    end
    
    pause(4)
    
    robotArm.setJointCommand('left', leftqs_fridge2);
    pause(0.5);
    while ~prod(abs(robotArm.joint_velocities) < 0.03); end
    pause(0.5);
    
    load('/home/cats/Douglas Robots/left_init.mat');
    % load('/home/cats/Douglas Robots/right_init.mat');

    % initial position affects the result of IK during the process a lot...
    robotArm.setPositionModeSpeed(0.15);
    robotArm.setJointCommand('left', left);
    % robotArm.setJointCommand('right', right);
    doorOperationPosition(robotArm, 0.385, 'right', -1)
    adjustPosePure( robotArm, robotPeripheries, 'right' )
    
    pos = robotArm.getPositions('r');
    ori = robotArm.getOrientations('r');
    pos = pos + [-0.08; -0.17; -0];
    rightqs_down = robotArm.solveIKfast(pos, ori, 'right');
    if ~isempty(leftqs_fridge)
        robotArm.setJointCommand('right', rightqs_down);
        pause(0.5);
        while ~prod(abs(robotArm.joint_velocities) < 0.03); end
        pause(3);
    end
    robotPeripheries.openGripper('r');

end


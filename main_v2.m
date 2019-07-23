% open the doors and transfer the food container
    % The following script makes the baxter on wheel open the fridge door
    % and take out a food container and put it into the microwave oven when
    % it is at the approximately-correct position

%% initialize all the parameter and device
% connecting to robotRR bridges

robotArm = RobotRaconteur.Connect('tcp://192.168.1.134:2345/BaxterJointServer/Baxter');
% robotArm.publishPoints(points)
rightCamera = RobotRaconteur.Connect('tcp://192.168.1.134:4567/BaxterCameraServer/right_hand_camera');
leftCamera = RobotRaconteur.Connect('tcp://192.168.1.134:9087/BaxterCameraServer/left_hand_camera');
robotPeripheries = RobotRaconteur.Connect('tcp://192.168.1.134:6708/BaxterPeripheralServer/BaxterPeripherals');
% set the mode=1 if using moveit module to move the arm.
% robotArm.setControlMode(uint8(1));
% robotArm.moveitSetJointCommand('both_arm', [1.0; 2.; 3.; 4.;5]);

% waiting for the initialization of the cameras
rightCamera.openCamera();
leftCamera.openCamera();
pause(3.5); 

load('cameraparams_right.mat');
load('cameraparams_left.mat');
load('Hubert.mat')
load('RoadMap.mat')
rightCamera.setCameraIntrinsics(cameraparams_right);
leftCamera.setCameraIntrinsics(cameraparams_lefthand);

% Make sure the marker size is correct
rightCamera.setMarkerSize(0.055);
leftCamera.setMarkerSize(0.055);
pause(1);

robotArm.setControlMode(uint8(1));
robotPeripheries.calibrateGripper('l');
robotPeripheries.openGripper('l');

robotPeripheries.calibrateGripper('r');
robotPeripheries.openGripper('r');
pause(1);

% set joint angle control mode 
robotArm.setControlMode(uint8(0));
% load('/home/cats/Douglas Robots/left_init.mat');
% load('/home/cats/Douglas Robots/right_init.mat');

% initial position affects the result of IK during the process a lot...
robotArm.setPositionModeSpeed(0.3);
% robotArm.setJointCommand('left', left);
% robotArm.setJointCommand('right', right);

% waiting for movements complete
% pause(1);
% while ~prod(abs(robotArm.joint_velocities) < 0.1); end
% pause(0.5);

% some data
% The process of opening the fridge door is divided into nine parts.
% Every part moves the door open 10 degrees.
deltaTheta = 10/180*pi;

deltaAlpha = -pi/2/12;
robotArm.setPositionModeSpeed(0.25);
r = 0.387; % The radius of the fridge door
r_m = 0.295;
l_grip = 0.085;
r_microwave = 0.295; % radius of the microwave door

% roadmap construction


for i = 4:4
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
    % this is a 4x4 matrix
    moveforward = [axang2rotm([1 0 0 -pi/16]), ...
        [0 -0.45 0.675+0.8/pi*abs(headPan)]';...
        0 0 0 1]; % pi/16 offsets the looking down angle of the head pan 
    
    Hbase2headside = ... 
        Hbase2headcam*moveforward;
    
    % position and otientation before the fridge door
    fridge_door_pos = Hbase2headside(1:3,4);
    fridge_door_ori = rotm2quat(Hbase2headside(1:3,1:3))';
    % init_qs = robotArm.solveIKfast(fridge_door_pos, fridge_door_ori, 'right');
    init_ptr = RoadMap('init_ptr');
    robotArm.setJointCommand('right', init_ptr.qs);
    rightqs_copy = init_ptr.qs;
    pause(0.5);
    while ~prod(robotArm.joint_velocities < 0.05); end
    pause(5);
    tic;
    arTagposes = rightCamera.ARtag_Detection();
    
    disp(arTagposes)
    while(toc < 0.1); pause(0.005); end
    
    % if no arTag, to go to the next iteration. Currently, this code is 
    % unnecessary. make sure the camera sees the tag of fridge
    if isempty(arTagposes)
        continue;
    end
    disp(arTagposes.ids)
    % current jointPosition
    % demoSearchFridge(arTagposes)
    % robotArm.setJointCommand('right', rightqs_copy);
    pause(0.5)
    while ~prod(abs(robotArm.joint_velocities) < 0.05); end
    pause(0.5);
    % 5 is upper fridge handle
    % 0 is lower fridge handle
    while( length(unique(arTagposes.ids))<3)
        arTagposes = rightCamera.ARtag_Detection();
        pause(0.3)
    end
    % RoadMap, points = processRoadMap(RoadMap, arTagposes);
    index_f = find(arTagposes.ids == 5); % upper fridge handle
    while isempty(index_f)
        arTagposes = rightCamera.ARtag_Detection();
        index_f = find(arTagposes.ids == 5);
        pause(0.3);
        disp 'find front door tag'
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% 2: move the right hand to the door handle %%%%%%%%%%%%%%%%
    % receive the transform from right hand to tag 
    my_tmats = arTagposes.tmats((index_f-1)*16+1: index_f*16);
    right2tag = reshape(my_tmats, 4, 4);
    % transformation matrix for the coordinates
    my_rotm = axang2tform([1 0 0 0.5*pi]) * axang2tform([0 1 0 -0.5*pi]);
    base2tag = my_rotm * right2tag;
    % the follow calculation is the orientation of current gripper
    base2tag(1:3, 1:3) = axang2rotm([1 0 0 0.5*pi]) * base2tag(1:3, 1:3);
    base2tag(1:3, 4) = -base2tag(1:3, 4) + robotArm.getPositions('r');

    position = base2tag(1:3,4) + [-l_grip; 0.078; -0.02];
    orientation = rotm2quat(base2tag(1:3,1:3))';
    % the position of the area at the front of the fridge.
    % for left hand
    front_fridge_pos = Hbase2headside(1:3,4) + [0.115; 0.06; 0];
    robotArm.setControlMode(uint8(0));
    rightqs = robotArm.solveIKfast(position, orientation, 'right');
    robotArm.setJointCommand('right', rightqs);
    pause(1)
    while ~prod(abs(robotArm.joint_velocities) < 0.05); end
    pause(1);
    robotPeripheries.closeGripper('r');
    %add_fridge(robotArm, base2tag);

    tempPos = robotArm.endeffector_positions;
    adjustPose( robotArm, robotPeripheries, 'right', [1; 1; 1] );

%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% 3: close the gripper and open the fridge door %%%%%%%%%%
    % tear down the process of drawing an arc into nine parts
    robotArm.setControlMode(uint8(3));
    robotArm.setPositionModeSpeed(0.05);

    doorOperationPosition2(robotArm, r, 'right')
    doorOperationPosition2(robotArm, r, 'right', -1)

    robotArm.setPositionModeSpeed(0.3);
    robotArm.removeAttachedObject('right_gripper', '')
    % robotPeripheries.openGripper('r');
    pause(1);
    % demoMoveWater();
    
    robotArm.setPositionModeSpeed(0.15);
    robotArm.setJointCommand('left', left);
    doorOperationPosition(robotArm, r, 'right', -1)

    pause(0.5);
    while ~prod(abs(robotArm.joint_velocities) < 0.03); end
    pause(1);
    
    % move the righthand backward
    % base2right = robotPeripheries.lookUptransforms('/base', ...
    %     '/right_hand');
    % Hbase2right = quat2tform([base2right.quaternion(4); ...
    %     base2right.quaternion(1:3)]');
    % Hbase2right(1:3,4) = base2right.position;
    % Hbase2idle = ...
    %     Hbase2right*[eye(3), [-0.1, 0, 0.15]'; 0, 0, 0, 1];
    % position = Hbase2idle(1:3,4);
    % orientation = rotm2quat(Hbase2idle(1:3,1:3))';
    % rightqs_b = robotArm.solveIKfast(position, orientation, 'right');
    % if ~isempty(rightqs_b)
    %     robotArm.setJointCommand('right', rightqs_b);
    % end
    % pause(0.5);
    % while ~prod(robotArm.joint_velocities < 0.05); end
    % pause(0.5);

%%
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
    leftqs_microwave = solveIK(robotArm, position, orientation, 'left');
    robotArm.setJointCommand('left', leftqs_microwave);

    pause(0.5);
    while ~prod(abs(robotArm.joint_velocities) < 0.03); end
    pause(3);

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
    % leftqs_microwave
    leftqs_m = robotArm.solveIKfast(position, orientation, 'left');
    if ~isempty(leftqs_m)
        robotArm.setJointCommand('left', leftqs_m);
    else
        disp('need help')
        input('enter when finish')
    end
    pause(1);
    while ~prod(robotArm.joint_velocities < 0.03); end
    pause(1);
    % robotPeripheries.closeGrippers('l');
    robotPeripheries.suck('l', uint8(100));
    pause(1);
    attachVacuumGripper(robotPeripheries, robotArm, 'left');
    pause(1);
    add_microwave(robotArm, Hbase2leftcam * left2tag);

%% the open microwave oven door
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%% 5: open the microwave oven door %%%%%%%%%%%%%%%%%%%%%
    % the process is divided into 12 parts
    robotArm.setPositionModeSpeed(0.15); % slowly (unnecessary...)
    % doorOperationPosition(robotArm, r_m, 'left' )

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
            position_temp = position + (rand(3,1)/500 - 0.001);
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
    robotArm.removeAttachedObject('left_gripper', '')

    robotPeripheries.openGripper('l');
    while ~prod(robotPeripheries.vacuum_sensor_value < 20); end
    pause(1);

    %% move the lefthand backward and downward to avoid collision
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
    while ~prod(abs(robotArm.joint_velocities) < 0.05); end
    pause(1.5);

    %% move the arm in front of the fridge
    % do not ues setJointCommand any more
    robotArm.setControlMode(uint8(1));
    leftqs_fridge = robotArm.solveIKfast(fridge_door_pos, fridge_door_ori, 'left');
    robotArm.moveitSetJointCommand('left', leftqs_fridge);
    pause(0.5);
    while ~prod(abs(robotArm.joint_velocities) < 0.05); end
    pause(3);

    %% 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% 6: detect the food container %%%%%%%%%%%%%%%%%%%%%%
%             if ~isempty(leftqs)
%                 % try to place left hand in front of the fridge
%                 robotArm.setJointCommand('left', leftqs);
%             end
%             pause(1);
%             while ~prod(robotArm.joint_velocities < 0.01); end
%             pause(2);

    % detection
    %system('say "look"');
    while 1
        arTagposes_l = leftCamera.ARtag_Detection();
        if isempty(arTagposes_l)
            system('spd-say "no tag detected"');
            continue;
        end

        index_t = find(arTagposes_l.ids == 6); % thing tag
        if isempty(index_t)
            system('spd-say "nothing detected"');
            disp('nothing detected');
            continue;
        end
        break
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%% 7: suck on the container %%%%%%%%%%%%%%%%%%%%%%%%
    % get base to left hand camera transform
    base2leftcam = robotPeripheries.lookUptransforms('/base', ...
        '/left_hand_camera');
    Hbase2leftcam = quat2tform([base2leftcam.quaternion(4); ...
        base2leftcam.quaternion(1:3)]');
    Hbase2leftcam(1:3,4) = base2leftcam.position;

    % receive the transform from left hand to tag 
    left2tag = reshape(arTagposes_l.tmats((index_t-1)*16+1: index_t*16), 4, 4);
    bias_l = axang2tform([0 1 0 pi]) * axang2tform([0 0 1 pi]);
    bias_l(1:3,4) = [0 -0.02 0.1]';

    % We are gonna move the hand to the container
    base2tag = Hbase2leftcam * left2tag * bias_l;
    position = base2tag(1:3,4);
    orientation = rotm2quat(base2tag(1:3,1:3))';

    % solve IK for joint angles that move the gripper to the food
    % thing
    robotArm.setPositionModeSpeed(0.15);
    robotArm.setControlMode(uint8(1));
    leftqs2 = robotArm.solveIKfast(position, orientation, 'left');
    if ~isempty(leftqs2)
        robotArm.moveitSetJointCommand('left', leftqs2);
    end
    pause(1);
    robotPeripheries.suck('l', uint8(100));
    pause(1);
    attachVacuumGripper(robotPeripheries, robotArm, 'left');
    pause(1);
    robotArm.setPositionModeSpeed(0.3);



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% 9: move left hand up in front of the microwave door %%%%%%%%%%%            
    bias_d = axang2tform([0, 0, 1, pi]);
    bias_d(1:3, 4) = [0.1 0.03 -0.1]';
    base2door = base2door * bias_d;
    position = base2door(1:3,4);
    orientation = rotm2quat(base2door(1:3,1:3))';

    % solve IK for joint angles that move the gripper to the door
    leftqs_u = robotArm.solveIKfast(position, orientation, 'left');
    if ~isempty(leftqs_u)
        robotArm.setJointCommand('left', leftqs_u);
    end
    pause(0.5);
    while ~prod(robotArm.joint_velocities < 0.05); end
    pause(1.5);
    robotArm.setPositionModeSpeed(0.5);

    % move right hand forward to close the fridge door
    base2righthand = robotPeripheries.lookUptransforms('/base', ...
        '/right_hand');
    Hbase2righthand = quat2tform([base2righthand.quaternion(4); ...
        base2righthand.quaternion(1:3)]');
    Hbase2righthand(1:3,4) = base2righthand.position;
    forwardbias = [eye(3), [-0.05 0 0.5]'; 0 0 0 1];
    Hforward = Hbase2righthand*forwardbias;
    position = Hforward(1:3,4);
    orientation = rotm2quat(Hforward(1:3,1:3))';
    rightqs_r = robotArm.solveIKfast(position, orientation, 'right');
    if ~isempty(rightqs_r)
        robotArm.setJointCommand('right', rightqs_r);
    end
    pause(1);
    while ~prod(robotArm.joint_velocities < 0.01); end

    robotArm.setPositionModeSpeed(0.15);

    % move left hand into the microwave oven
    base2lefthand = robotPeripheries.lookUptransforms('/base', ...
        '/left_hand');
    Hbase2lefthand = quat2tform([base2lefthand.quaternion(4); ...
        base2lefthand.quaternion(1:3)]');
    Hbase2lefthand(1:3,4) = base2lefthand.position;
    forwardbias = [eye(3), [0 0 0.3]'; 0 0 0 1];
    Hforward = Hbase2lefthand*forwardbias;
    position = Hforward(1:3,4);
    orientation = rotm2quat(Hforward(1:3,1:3))';
    leftqs_f = robotArm.solveIKfast(position, orientation, 'left');
    if ~isempty(leftqs_f)
        robotArm.setJointCommand('left', leftqs_f);
    end
    pause(0.5);
    while ~prod(robotArm.joint_velocities < 0.01); end
    pause(0.5);
    robotPeripheries.openGripper('l');
    robotArm.setPositionModeSpeed(0.3);

    break
end
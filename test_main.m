% open the doors and transfer the food container
    % The following script makes the baxter on wheel open the fridge door
    % and take out a food container and put it into the microwave oven when
    % it is at the approximately-correct position
    %


% clear
% rosshutdown
% rosinit('011303P0004.local','NodeHost','192.168.1.134');


% connecting to robotRR bridges
robotArm = RobotRaconteur.Connect('tcp://localhost:2345/BaxterJointServer/Baxter');
rightCamera = RobotRaconteur.Connect('tcp://localhost:4567/BaxterCameraServer/right_hand_camera');
leftCamera = RobotRaconteur.Connect('tcp://localhost:9087/BaxterCameraServer/left_hand_camera');
robotPeripheries = RobotRaconteur.Connect('tcp://localhost:6708/BaxterPeripheralServer/BaxterPeripherals');

% set the mode=1 if using moveit module to move the arm.
% robotArm.setControlMode(uint8(1));
% robotArm.moveitSetJointCommand('both_arm', [1.0; 2.; 3.; 4.;5]);

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

robotPeripheries.calibrateGripper('r');
robotPeripheries.openGripper('r');
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
while ~prod(robotArm.joint_velocities < 0.1); end%     img = readImage(imgMsg);
%     imshow(img);
pause(0.5);

% some data
deltaTheta = 10/180*pi;
    % The process of opening the fridge door is divided into nine parts.
    % Every part moves the door open 10 degrees.
deltaAlpha = -pi/2/12;
robotArm.setPositionModeSpeed(0.3);
r = 0.25; % The radius of the fridge door

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
    position = Hbase2headside(1:3,4);
    orientation = rotm2quat(Hbase2headside(1:3,1:3))';
    rightqs = robotArm.solveIKfast(position, orientation, 'right');
    robotArm.setJointCommand('right', rightqs);
    
    pause(0.5);
    while ~prod(robotArm.joint_velocities < 0.05); end
    pause(10);
    tic;
    arTagposes = rightCamera.ARtag_Detection();
    
%     imgSub = rossubscriber('/cameras/right_hand_camera/image');
%     imgMsg = receive(imgSub); % or imgMsg = imgSub.LatestMessage
%     img = readImage(imgMsg);
%     imshow(img);

    disp('Check whether find the tag or not')
    disp(arTagposes)
    while(toc < 0.1); pause(0.005); end
    
    % make sure the camera sees the tag of fridge
    if isempty(arTagposes)
        continue;
    end
    disp(arTagposes.ids)
    index_f = find(arTagposes.ids == 5); % fridge handle
    if isempty(index_f)
        continue
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% 2: move the right hand to the door handle %%%%%%%%%%%%%%%%
    % get base to right hand camera transform
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
    bias(1:3, 4) = [-0.1 -0.025 -0.045]';
    bias = bias*axang2tform([0 0 1 pi])*axang2tform([1 0 0 -pi/36]);
    base2tag = Hbase2rightcam * right2tag * bias;
     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%  debug  %%%%
    %    testout = Hbase2rightcam * right2tag * ...
    %        axang2tform([1 0 0 -pi/2]) * axang2tform([0 0 1 pi/2])
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % middle grib position
    % position = base2tag(1:3,4) - [0.1201; 0.0058; 0.0627];
    position = base2tag(1:3,4);

    
    % the position of the area at the front of the fridge.
    % for left hand
    front_fridge_pos = Hbase2headside(1:3,4) + [0.1; 0; 0];
    orientation = rotm2quat(base2tag(1:3,1:3))';
    robotArm.setControlMode(uint8(0));
    rightqs = robotArm.solveIKfast(position, orientation, 'right');
    robotArm.setJointCommand('right', rightqs);
    pause(3);
    robotPeripheries.closeGripper('r');
    
    tempPos = robotArm.endeffector_positions;
    
    %     position(3) = position(3) + 0.005;
    %     rightqs = robotArm.solveIKfast(position, orientation, 'right');
    %     robotArm.setJointCommand('right', rightqs);
    %     pause(0.5)
    %     position(3) = position(3) - 0.01;
    %     rightqs = robotArm.solveIKfast(position, orientation, 'right');
    %     robotArm.setJointCommand('right', rightqs);
    %     pause(0.5)
    %     position(3) = position(3) + 0.005;
    %     rightqs = robotArm.solveIKfast(position, orientation, 'right');
    %     robotArm.setJointCommand('right', rightqs);        
    %     pause(0.5)
    
    %% check right hand hold the door
    % move left camera
%     load('important_data.mat', 'obv_pos')
%     load('important_data.mat', 'obv_ori')
%     
%     leftqs_check = robotArm.solveIKfast(obv_pos, obv_ori, 'left');
%     robotArm.setJointCommand('left', leftqs_check);        
%     pause(3)
%     
% 
%     
%     load('Hubert.mat', 'record');
%     % to move arm bu user
%     % moveArm(robotArm, 'right')
%     followPath(robotArm, record );
%     
%     record = zeros(14,100);
%     
%     % move left hand to the front of fridge
%     front_fridge_ori = orientation;
%     % front_fridge_ori(1:4) = [ 0.7189; -0.0079; 0.6934; -0.0473 ];
%     left_front_fridge_qs = robotArm.solveIKfast(front_fridge_pos, front_fridge_ori, 'left');
%     robotArm.setJointCommand('left', left_front_fridge_qs);
%     load( 'detection.mat', 'record' );
%     ArTags = searchFood( leftCamera, robotArm, record );
%     getFood(robotArm, robotPeripheries, ArTags, 'left');

    
%     for i = 1:100
%         pause(0.2);
%         record(:,i) = robotArm.joint_positions;
%     end
%     disp(record)
    %%Hubert test
%     
%     
%     rightqs1 = robotArm.solveIKfast(position, orientation, 'right');
%     while isempty(rightqs1)
%     % solve IK for joint angles that move the gripper to the fridge handle
%         rightqs1 = calculate_IK(rightCamera, robotArm, robotPeripheries);
%         pause(1);
%     end
%     
%     % we need to put left hand in front of the fridge before taking things
%     % out. Calculate the joint angles.
%     biasl = axang2tform([0 1 0 pi]);
%     biasl(1:3, 4) = [-0.1 0.1 0.19]';
%     biasl = biasl * axang2tform([0 0 1 pi]);
%     base2tagl = Hbase2rightcam * right2tag * biasl;
%     positionl = base2tagl(1:3,4);
%     orientationl = rotm2quat(base2tagl(1:3,1:3))';
%     
%     % solve IK for joint angles that move the vacuum on left hand in front 
%     % of the fridge
%     leftqs = robotArm.solveIKfast(positionl, orientationl, 'left');
%     disp(rightqs1)
     if 1
%      if ~isempty(rightqs1)
%         % move right hand in front of the handle
%         robotArm.setJointCommand('right', rightqs1);
%         pause(1);
%         while ~prod(robotArm.joint_velocities < 0.1); end
%         pause(0.5);
%         
%         % push the right hand forward a little bit to grip the handle
%         base2right = robotPeripheries.lookUptransforms('/base', ...
%             '/right_hand');
%         Hbase2right = quat2tform([base2right.quaternion(4); ...
%             base2right.quaternion(1:3)]');
%         Hbase2right(1:3,4) = base2right.position;
%         base2handle = Hbase2right * [eye(3), [0 0 0.2]'; 0 0 0 1];
%         position2 = base2handle(1:3,4);
%         orientation2 = rotm2quat(base2handle(1:3,1:3))';
%         rightqs2 = robotArm.solveIKfast(position2, orientation2, 'right');
       if 2
%          if ~isempty(rightqs2)
%             robotArm.setJointCommand('right', rightqs2);
%             pause(0.5);
%             while ~prod(robotArm.joint_velocities < 0.05); end
%             pause(0.5);
%             robotPeripheries.closeGripper('r');
%             pause(1);
%             
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% 3: close the gripper and open the door %%%%%%%%%%%%%%%%%
            % tear down the process of drawing an arc into nine parts
            robotArm.setPositionModeSpeed(0.15); % slowly (unnecessary...)
            robotPeripheries.closeGripper('r');
            for j = 1:9
                base2right = robotPeripheries.lookUptransforms('/base', ...
                    '/right_hand');
                Hbase2right = quat2tform([base2right.quaternion(4); ...
                    base2right.quaternion(1:3)]');
                Hbase2right(1:3,4) = base2right.position;
                base2handle = Hbase2right * ...
                    [axang2rotm([0 1 0 deltaTheta]), ...
                    [-r*(1-cos(deltaTheta)) 0 0.15-r*sin(deltaTheta)]'; ...
                    0 0 0 1];
                position_temp = base2handle(1:3,4);
                orientation_temp = rotm2quat(base2handle(1:3,1:3))';
                
                % each part requires an IK 
                rightqs_temp = robotArm.solveIKfast(position_temp, ...
                    orientation_temp, 'right');
                if ~isempty(rightqs_temp)
                    robotArm.setJointCommand('right', rightqs_temp);
                end
                pause(0.3);
                while ~prod(robotArm.joint_velocities < 0.06); end
                pause(0.3);
            end
            robotArm.setPositionModeSpeed(0.3);
                        
            robotPeripheries.openGripper('r');
            pause(1);
            
            % move the righthand backward
            base2right = robotPeripheries.lookUptransforms('/base', ...
                '/right_hand');
            Hbase2right = quat2tform([base2right.quaternion(4); ...
                base2right.quaternion(1:3)]');
            Hbase2right(1:3,4) = base2right.position;
            Hbase2idle = ...
                Hbase2right*[eye(3), [-0.1, 0, 0.15]'; 0, 0, 0, 1];
            position = Hbase2idle(1:3,4);
            orientation = rotm2quat(Hbase2idle(1:3,1:3))';
            rightqs_b = robotArm.solveIKfast(position, orientation, 'right');
            if ~isempty(rightqs_b)
                robotArm.setJointCommand('right', rightqs_b);
            end
            pause(0.5);
            while ~prod(robotArm.joint_velocities < 0.05); end
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
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% 5: open the microwave oven door %%%%%%%%%%%%%%%%%%%%%
            % the process is divided into 12 parts
            for j = 1:12
                arTagposes_l = leftCamera.ARtag_Detection();
                if isempty(arTagposes_l)
                    system('spd-say "no tag detected"');
                    continue;
                end

                index_m = find(arTagposes_l.ids == 2); % door tag
                if isempty(index_m)
                    system('spd-say "nothing detected"');
                    continue;
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
                bias_l = axang2tform([0 1 0 pi]);
                bias_l(1:3,4) = [0.087 0 0.12]';
                base2door_temp = Hbase2leftcam * left2tag * bias_l * ...
                    axang2tform([0 1 0 deltaAlpha]);
                position = base2door_temp(1:3,4);
                orientation = rotm2quat(base2door_temp(1:3,1:3))';

                % solve IK for joint angles that move the gripper
                leftqs_temp = robotArm.solveIKfast(position, ...
                    orientation, 'left');
                if ~isempty(leftqs_temp)
                    robotArm.setJointCommand('left', leftqs_temp);
                end
                pause(0.2);
                while ~prod(robotArm.joint_velocities < 0.06); end
                pause(0.3);
            end
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
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%% 6: detect the food container %%%%%%%%%%%%%%%%%%%%%%
            if ~isempty(leftqs)
                % try to place left hand in front of the fridge
                robotArm.setJointCommand('left', leftqs);
            end
            pause(1);
            while ~prod(robotArm.joint_velocities < 0.01); end
            pause(2);
            
            % detection
            system('say "look"');
            arTagposes_l = leftCamera.ARtag_Detection();
            if isempty(arTagposes_l)
                system('spd-say "no tag detected"');
                continue;
            end

            index_t = find(arTagposes_l.ids == 6); % thing tag
            if isempty(index_t)
                system('spd-say "nothing detected"');
                continue;
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
            leftqs2 = robotArm.solveIKfast(position, orientation, 'left');
            if ~isempty(leftqs2)
                robotArm.setJointCommand('left', leftqs2);
            end
            pause(2);
            robotPeripheries.closeGripper('l');
            while ~prod(robotArm.joint_velocities < 0.01); end
            pause(2);
            robotPeripheries.closeGripper('l');
            pause(1);
            robotArm.setPositionModeSpeed(0.3);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%% 8: move left hand out of the fridge %%%%%%%%%%%%%%%%%%%
            for l = 1:2
                base2lefthand = robotPeripheries.lookUptransforms('/base', ...
                    '/left_hand');
                Hbase2lefthand = quat2tform([base2lefthand.quaternion(4); ...
                    base2lefthand.quaternion(1:3)]');
                Hbase2lefthand(1:3,4) = base2lefthand.position;
                backbias = [eye(3), [-0.01 0 -0.05]'; 0 0 0 1];
                Hbackward = Hbase2lefthand*backbias;
                position = Hbackward(1:3,4);
                orientation = rotm2quat(Hbackward(1:3,1:3))';
                leftqs_b = robotArm.solveIKfast(position, ...
                    orientation, 'left');
                if ~isempty(leftqs_b)
                    robotArm.setJointCommand('left', leftqs_b);
                end
                pause(0.5);
                while ~prod(robotArm.joint_velocities < 0.05); end
                pause(0.5);
            end
            pause(1);
            
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
        end
        break
    else 
    end
end
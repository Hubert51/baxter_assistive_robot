function [ output_args ] = demoMoveWater(  )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    side = 'left';
    robotArm = RobotRaconteur.Connect('tcp://localhost:2345/BaxterJointServer/Baxter');
    rightCamera = RobotRaconteur.Connect('tcp://localhost:4567/BaxterCameraServer/right_hand_camera');
    leftCamera = RobotRaconteur.Connect('tcp://localhost:9087/BaxterCameraServer/left_hand_camera');
    robotPeripheries = RobotRaconteur.Connect('tcp://localhost:6708/BaxterPeripheralServer/BaxterPeripherals');
    robotArm.setPositionModeSpeed(0.15); % slowly (unnecessary...)
    robotArm.setControlMode(uint8(0));
    load('Hubert1.mat')
    leftqs_fridge = robotArm.solveIKfast(fridge_door_pos, fridge_door_ori2, 'left');
    if ~isempty(leftqs_fridge)
        robotArm.setJointCommand('left', leftqs_fridge2);
    end

    pause(0.5);
    while ~prod(abs(robotArm.joint_velocities) < 0.03); end
    pause(3);

    % detect the microwave tag
    arTagposes_l = leftCamera.ARtag_Detection();
    arTagposes_l.ids
    if isempty(arTagposes_l)
        disp('spd-say "no tag detected"');
    end

    index_m = find(arTagposes_l.ids == 8); % door tag
    if isempty(index_m)
        disp('spd-say "nothing detected"');
    end
    
    q2 = robotArm.getOrientations('l');
    R2 = quat2rotm(q2');
%     left2tagWater = reshape(arTagposes_l.tmats((index_m-1)*16+1: index_m*16), ...
%         4, 4);
%     pos = robotArm.getPositions('left') + left2tagWater(1:3,4);
%     pos(3) = pos(3) - 0.25;
%     leftqs_fridge = robotArm.solveIKfast(pos, fridge_door_ori, 'left');
%     if ~isempty(leftqs_fridge)
%         robotArm.moveitSetJointCommand('left', leftqs_fridge);
%     end
    
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
    bias_l(1:3,4) = [0.09 -0.2 0.1]';
    base2door = Hbase2leftcam * left2tag * bias_l;
    position = robotArm.getPositions('l') + R2 * left2tag(1:3,4);
    position = position + [-0.1; -0.072; -0.07];
    orientation = rotm2quat(base2door(1:3,1:3))';
    % solve IK for joint angles that move the gripper to the place
    % leftqs_microwave
    leftqs_m = robotArm.solveIKfast(position, orientation, 'left');
    if ~isempty(leftqs_m)
        robotArm.setJointCommand('left', leftqs_m);
    end
    pause(0.5)
    while ~prod(abs(robotArm.joint_velocities) < 0.05); end
    pause(0.5);
    robotPeripheries.suck('l', uint8(1000));
    attachVacuumGripper(robotPeripheries, robotArm, side)

    % move a little up
    position = position + [0; 0; 0.02];
    leftqs_m = robotArm.solveIKfast(position, orientation, 'left');
    if ~isempty(leftqs_m)
        robotArm.setJointCommand('left', leftqs_m);
    end
    pause(0.5)
    while ~prod(abs(robotArm.joint_velocities) < 0.05); end
    pause(0.5);

    if ~isempty(leftqs_fridge2)
        robotArm.setJointCommand('left', leftqs_fridge2);
        pause(0.5)
        while ~prod(abs(robotArm.joint_velocities) < 0.05); end
        pause(0.5);
    end
    
    
    pos_above = fridge_door_pos + [0; 0; 0.585];
    leftqs_above = robotArm.solveIKfast(pos_above, fridge_door_ori2, 'left');
    if ~isempty(leftqs_fridge)
        robotArm.setJointCommand('left', leftqs_above);
        pause(0.5);
        while ~prod(abs(robotArm.joint_velocities) < 0.03); end
        pause(3);
    end
    
    pos_above = pos_above + [0.25; 0; 0];
    fridge_door_ori2 = [0.0350; 0.6464; 0.0020; 0.7622];
    leftqs_above = robotArm.solveIKfast(pos_above, fridge_door_ori2, 'left');
    if ~isempty(leftqs_fridge)
        robotArm.setJointCommand('left', leftqs_above);
        pause(0.5);
        while ~prod(abs(robotArm.joint_velocities) < 0.03); end
        pause(0.5);
    end
    
    % transfer the water
    leftqs_above(7) = leftqs_above(7) - 90*pi/180;
    robotArm.setPositionModeSpeed(0.1); 
    if ~isempty(leftqs_fridge)
        robotArm.setJointCommand('left', leftqs_above);
        pause(0.5);
        while ~prod(abs(robotArm.joint_velocities) < 0.03); end
        pause(0.5);
    end
    
    pause(4)
    
    % finish transfering the water
    leftqs_above(7) = leftqs_above(7) + 90*pi/180;
    if ~isempty(leftqs_fridge)
        robotArm.setJointCommand('left', leftqs_above);
        pause(0.5);
        while ~prod(abs(robotArm.joint_velocities) < 0.03); end
        pause(0.5);
    end

    pos_above = pos_above + [0; -0.05; -0.07];
    leftqs_above = robotArm.solveIKfast(pos_above, fridge_door_ori2, 'left');
    if ~isempty(leftqs_above)
        robotArm.setJointCommand('left', leftqs_above);
        pause(0.5);
        while ~prod(abs(robotArm.joint_velocities) < 0.03); end
        pause(0.5);
    end
    
    robotPeripheries.openGripper('l')
    pos_above = pos_above + [-0.15; 0; 0];
    leftqs_above = robotArm.solveIKfast(pos_above, fridge_door_ori2, 'left');
    if ~isempty(leftqs_above)
        robotArm.setJointCommand('left', leftqs_above);
        pause(0.5);
        while ~prod(abs(robotArm.joint_velocities) < 0.03); end
        pause(0.5);
    end
    


    
    
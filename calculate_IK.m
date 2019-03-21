function [ rightqs1 ] = calculate_IK( rightCamera, robotArm, robotPeripheries )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    
    arTagposes = rightCamera.ARtag_Detection();
    if isempty(arTagposes)
        rightqs1 = int16([]);

        return;
    end
    
    disp(arTagposes.ids);
    index_f = find(arTagposes.ids == 2); % fridge handle
    if isempty(index_f)
        rightqs1 = int16([]);
        return
    end
    disp('find the tag!');
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
    bias(1:3, 4) = [-0.1 -0.025 0.04]';
    bias = bias*axang2tform([0 0 1 pi])*axang2tform([1 0 0 -pi/36]);
    base2tag = Hbase2rightcam * right2tag * bias;
    
    position = base2tag(1:3,4);
    orientation = rotm2quat(base2tag(1:3,1:3))';
    disp('finish calculate the inverse kinematics')
    rightqs1 = robotArm.solveIKfast(position, orientation, 'right');
    
    
end


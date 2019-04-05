

%% adjust the position of the robotArm

index_f = find(arTagposes.ids == 6); % fridge handle

% get base to left hand camera transform
base2leftcam = robotPeripheries.lookUptransforms('/base', ...
    '/left_hand_camera');
Hbase2leftcam = quat2tform([base2leftcam.quaternion(4); ...
    base2leftcam.quaternion(1:3)]');
Hbase2leftcam(1:3,4) = base2leftcam.position;

% receive the transform from right hand to tag 
left2tag = reshape(arTagposes.tmats((index_f-1)*16+1: index_f*16), ...
    4, 4);

% the position we are gonna put the hand in front of the handle has a 
% bias from the tag. 
bias = axang2tform([1 0 0 -pi]);
bias(1:3, 4) = [ 0 0 0 ]';
bias = bias*axang2tform([0 0 1 pi])*axang2tform([1 0 0 -pi/36]);
base2foodtag = Hbase2leftcam * left2tag * bias;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  debug  %%%%
%    testout = Hbase2rightcam * right2tag * ...
%        axang2tform([1 0 0 -pi/2]) * axang2tform([0 0 1 pi/2])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% middle grib position
position = base2foodtag(1:3,4) - [0.1201; 0.0058; 0.0627];
orientation = rotm2quat(base2foodtag(1:3,1:3))';
leftqs = robotArm.solveIKfast(positionl, orientationl, 'left');
robotArm.setJointCommand('left', leftqs);   

%% grab the food
vacuum_value = robotPeripheries.vacuum_data;
while vacuum_value < 50
    if strcmp( side, 'left')
        position = position(1:3);
        orientation = orientation(1:4);
    else
        position = position(4:6);
        orientation = orientation(5:8);
    end



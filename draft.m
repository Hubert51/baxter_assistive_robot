% function [ result ] = check_diff_state( old, new, data )
% %UNTITLED2 Summary of this function goes here
% %   Detailed explanation goes here
%     old = 
%     diff = sum((new - old).^2);
%     while
% 
% end
% 
% old = record(:,1) ;
% for i = 1:100
%     new = record(:,i);
%     diff = sum( (new-old).^2 );
%     disp(diff);
%     old = new;
%     pause(0.5);
% end

% img = leftCamera.getCurrentImage();
% b = reshape(img.data(1:4:end), img.width, img.height );
% g = reshape(img.data(2:4:end), img.width, img.height );
% r = reshape(img.data(3:4:end), img.width, img.height );
% im = cat(3, r,g,b);
% im = imrotate(im, 270);
% imshow(im)

% 
% fridge_door_pos = [0.7720; 0.0093; 0.1862];
% fridge_door_ori = [0.5008; 0.4926; 0.4991; 0.5074];
% leftqs_fridge_door = robotArm.solveIKfast(fridge_door_pos, fridge_door_ori, 'left');
% robotArm.setJointCommand('left', leftqs_fridge_door);
% 
% pause(0.5);
% while ~prod(robotArm.joint_velocities < 0.01); end
% pause(2);

% system('say "look"');

%% rotate the arm
offset = 0;
position = robotArm.endeffector_positions;

position = position(4:6);

base2right = robotPeripheries.lookUptransforms('/base', ...
                    '/right_hand');
Hbase2right = quat2tform([base2right.quaternion(4); ...
    base2right.quaternion(1:3)]');
Hbase2right(1:3,4) = base2right.position;

%                 original parameter
base2handle = Hbase2right * ...
    [axang2rotm([0 1 0 deltaTheta]), ...
    [-r*(1-cos(deltaTheta)) 0 r*sin(deltaTheta)+offset]'; ...
    0 0 0 1];

% new parameter
%                 base2handle = Hbase2right * ...
%                     [axang2rotm([0 1 0 deltaTheta]), ...
%                     [-r*(1-cos(deltaTheta)) 0 r*sin(deltaTheta)]'; ...
%                     0 0 0 1];

% the position from homogenous transformation
% position_temp = base2handle(1:3,4);
orientation_temp = rotm2quat(base2handle(1:3,1:3))';

% each part requires an IK 
rightqs_temp = robotArm.solveIKfast(position, ...
    orientation_temp, 'right');
if ~isempty(rightqs_temp)
    robotArm.moveitSetJointCommand('right', rightqs_temp);
end

%% explore twist and wrench
while 1
    wrenches = robotArm.endeffector_wrenches;
    wrenches = wrenches(7:12);

    position = robotArm.endeffector_positions;
    position = position(4:6);
    

    cross(position, wrenches(4:6));
    wrenches(4:6)
    pause(0.3)
end
return

%%

base2leftcam = robotPeripheries.lookUptransforms('base', '/left_hand_camera');
q2 = base2leftcam.quaternion;
%R2 = quat2rotm(q2')
q2 = [q2(4); q2(1:3)]'
R2 = quat2rotm(q2);
R2 * [0; 1; 0]



arTagposes_l = leftCamera.ARtag_Detection();
if isempty(arTagposes_l)
    system('spd-say "no tag detected"');
    disp 'no tag detected';
end

index_t = find(arTagposes_l.ids == 6); % thing tag
if isempty(index_t)
    system('spd-say "nothing detected"');
    disp 'nothing detected';
else
    disp 'detected food';
end



% get base to left hand camera transform
base2leftcam = robotPeripheries.lookUptransforms('/base', ...
    '/left_hand_camera');
Hbase2leftcam = quat2tform([base2leftcam.quaternion(4); ...
    base2leftcam.quaternion(1:3)]');
Hbase2leftcam(1:3,4) = base2leftcam.position;

% receive the transform from left hand to tag 
left2tag = reshape(arTagposes_l.tmats((index_t-1)*16+1: index_t*16), 4, 4);
bias_l = axang2tform([0 1 0 pi]) * axang2tform([0 0 1 pi]);
% bias_l(1:3,4) = [0 -0.02 0.05]';   % this is original way to add bias

% We are gonna move the hand to the container
base2tag = Hbase2leftcam * left2tag * bias_l;
position = base2tag(1:3,4);
orientation = rotm2quat(base2tag(1:3,1:3))';

% solve IK for joint angles that move the gripper to the food
% thing
robotArm.setPositionModeSpeed(0.15);
leftqs2 = robotArm.solveIKfast(position, orientation, 'left');
if ~isempty(leftqs2)
    robotArm.moveitSetJointCommand('left', leftqs2);
end

pause(0.5);
while ~prod(robotArm.joint_velocities < 0.01); end
pause(2);
robotPeripheries.closeGripper('l');
while ~prod(robotArm.joint_velocities < 0.01); end
pause(2);
robotPeripheries.closeGripper('l');
pause(1);
robotArm.setPositionModeSpeed(0.3);


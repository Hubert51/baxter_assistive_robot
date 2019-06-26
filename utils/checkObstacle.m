function [ result ] = checkObstacle(robotArm, robotPeripheries, side )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

result1 = robotPeripheries.getRangerValue(side);
result11 = robotArm.IR_values;
disp(result1)
side_camera = strcat( '/', side, '_hand_camera' );
my_base2cam = robotPeripheries.lookUptransforms('base', side_camera);
q2 = my_base2cam.quaternion;
q2 = [q2(4); q2(1:3)]';
R2 = quat2rotm(q2);
p = R2 * [0; 1; 0];
p = -p * 0.06;

pos = robotArm.endeffector_positions;
pos = pos(1:3) + p;
ori = robotArm.endeffector_orientations;
my_qs = robotArm.solveIKfast(pos, ori, side);
robotArm.setJointCommand(side, my_qs);
pause(0.5);
while ~prod(abs(robotArm.joint_velocities) < 0.05); end
pause(2);

result2 = robotPeripheries.getRangerValue(side);
result22 = robotArm.IR_values;
result = [result1, result2, result11(1), result22(1)];

% move arm back
pos = pos(1:3) - p;
my_qs = robotArm.solveIKfast(pos, ori, side);
robotArm.setJointCommand(side, my_qs);
pause(0.5);
while ~prod(abs(robotArm.joint_velocities) < 0.05); end
pause(0.5)



end


function [  ] = moveBack( robotPeripheries, robotArm, side )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

step_size = 0.03;
side_camera = strcat( '/', side, '_hand_camera' );
my_base2cam = robotPeripheries.lookUptransforms('base', side_camera);
q2 = my_base2cam.quaternion;
q2 = [q2(4); q2(1:3)]';
R2 = quat2rotm(q2);
% in the zero configuration, [0 0 1] is moving end_effector towards the
% direction of gripper.
p = R2 * [0; 0; -1];

p = p * step_size;

pos = robotArm.endeffector_positions;
pos = pos(4:6) + p;
ori = robotArm.endeffector_orientations;
ori = ori(5:8);
my_qs = robotArm.solveIKfast(pos, ori, side);
robotArm.setJointCommand(side, my_qs);
pause(0.5);
while ~prod(abs(robotArm.joint_velocities) < 0.05); end
pause(1);



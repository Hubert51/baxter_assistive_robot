prev_torques = robotArm.joint_torques;
prev_torques = prev_torques(8:14);

while 1
    torques = robotArm.joint_torques;
    torques = torques(8:14);
    torques - prev_torques
    pause(2)
    prev_torques = torques;
end




% robotArm.endeffector_positions
% robotArm.endeffector_orientations
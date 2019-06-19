joint_pos = robotArm.joint_positions;
torque = robotArm.joint_torques;
pos_diff = position_temp - real_pos(4:6)
torque = torque(8:14)


%% code for recalculate the trajectory
while ~isempty(find( abs(joint_diff) > 0.7, 1))
    temp_offset = rand()*0.1 - 0.05;
    base2handle = Hbase2left * ...
    [axang2rotm([0 1 0 -deltaTheta]), ...
    [-r_m*(1-cos(deltaTheta)) 0 -r_m*sin(deltaTheta)+temp_offset]'; ...
    0 0 0 1];
    position_temp = base2handle(1:3,4);
    joint_pos = robotArm.joint_positions;
    joint_pos = joint_pos(1:7);
    leftqs_temp = robotArm.solveIKfast(position_temp, ...
    orientation_temp, 'left');
    joint_diff = leftqs_temp - joint_pos;
    disp 'recalculate trajectory';
end
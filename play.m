robot2.setJointCommand('left',record(1:7,1));
robot2.setJointCommand('right',record(8:14,1));
pause(2);
delta = [0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05 0.05]';
for i = 2:3:76
    endeffector = record(:, i);
    disp(endeffector(8:14));
    robot2.setJointCommand('left',endeffector(1:7));
    robot2.setJointCommand('right',endeffector(8:14));
    pause(0.1);
    while (prod(abs(endeffector - robot2.joint_positions) < delta) ~= 1) 
        % disp(i);
    end
end
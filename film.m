robot2 = RobotRaconteur.Connect('tcp://localhost:2345/BaxterJointServer/Baxter');
record = zeros(14,100);
for i = 1:100
    pause(0.2);
    record(:,i) = robot2.joint_positions;
end
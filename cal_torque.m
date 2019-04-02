robot2 = RobotRaconteur.Connect('tcp://localhost:2345/BaxterJointServer/Baxter');
old = robot2.joint_torques;
while 1
    pause(0.1)
    new = robot2.joint_torques;
    diff = sum((new - old).^2);
    disp(diff);
    
    old = new;
    
end
    
    
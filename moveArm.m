function [  ] = moveArm( robotArm, side )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    position = robotArm.endeffector_positions;
    orientation = (robotArm.endeffector_orientations);
    
    if strcmp( side, 'left')
        position = position(1:3);
        orientation = orientation(1:4);
    else
        position = position(4:6);
        orientation = orientation(5:8);
    end
    for i = 1:1000
        
    prompt = 'Please input 1~6 to move robot arm ';
    my_input = input(prompt);
    disp(my_input)

        if my_input == 1
            position(1) = position(1) + 0.01;
        elseif my_input == 2
            position(2) = position(2) + 0.01;
        elseif my_input == 3
            position(3) = position(3) + 0.01;
        elseif my_input == 4
            position(1) = position(1) - 0.01;

        elseif my_input == 5
            position(2) = position(2) - 0.01;
        elseif my_input == 6
            position(3) = position(3) - 0.01;  
        elseif my_input == 7
            save_data = robotArm.joint_positions;
            save('doorPos.mat', 'save_data')
            return
        end

        qs = robotArm.solveIKfast(position, orientation, side);
        if isempty(qs) 
            disp('no solution!')
            continue
        end
        robotArm.setJointCommand(side, qs);
    end

end


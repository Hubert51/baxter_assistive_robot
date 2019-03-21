function [  ] = changeOrientation( robotArm, side )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    orientations = robotArm.endeffector_orientations;
    positions = robotArm.endeffector_positions;
    
    if strcmp( 'left', side )
        orientations = orientations(1:4);
        positions = positions(1:3);
    else
        orientations = orientations(5:8);
        positions = positions(4:6);
    end

    for i = 1:1000
        
    prompt = 'Please input 1~8 to change robot arm orientations';
    my_input = input(prompt);
    disp(my_input)

        if my_input == 1
            orientations(1) = orientations(1) + 0.01;
        elseif my_input == 2
            orientations(2) = orientations(2) + 0.01;
        elseif my_input == 3
            orientations(3) = orientations(3) + 0.01;
        elseif my_input == 4
            orientations(4) = orientations(4) + 0.01;
        elseif my_input == 5
            orientations(1) = orientations(1) - 0.01;
        elseif my_input == 6
            orientations(2) = orientations(2) - 0.01;  
        elseif my_input == 7
            orientations(3) = orientations(3) - 0.01;
        elseif my_input == 8
            orientations(4) = orientations(4) - 0.01;  
        elseif my_input == 9
%             save_data = robotArm.joint_positions;
%             save('doorPos.mat', 'save_data')
            return
        end
        
        qs = robotArm.solveIKfast(positions, orientations, side');
        if isempty(qs) 
            disp('no solution!')
            continue
        end
        robotArm.setJointCommand(side, qs);
    end

end


function [ ArPoses ] = searchFood( camera, robotArm, record )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    for i = 1:length(record(1,:))
        endeffector = record(:, i);
        robotArm.setJointCommand('left',endeffector(1:7));
        robotArm.setJointCommand('right',endeffector(8:14));
        pause(1.5);
        FoodArTagPose = camera.ARtag_Detection();
        if isempty(FoodArTagPose)
            continue
        elseif isempty( find(FoodArTagPose.ids == 6, 1 ) )
            continue 
        else
            ArPoses = FoodArTagPose;
            break
        end
    end
end


function [ ] = add_microwave(robotArm, H_matrix )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    depth = 0.8;
    width = 0.475;
    height = 0.275;
    % add fridge into the rviz
    
    
    
    % front door
    center_y = H_matrix(2, 4) - 0.0275;
    center_z = H_matrix(3, 4) - 0.0125;
    center_x = H_matrix(1, 4) + 0.4;
    % not all the area of front is the door. so the width of door is
    
    
    % smaller than the total width
    temp_center_y = H_matrix(2, 4) + 0.015;
    temp_center_x = center_x - 0.5 * depth;
    
    robotArm.addBox('microwave_front', [0.01; 0.395; height], [temp_center_x; temp_center_y; center_z]);
    robotArm.attachBox('left_gripper', 'microwave_front');    
    
    % left
    left_y = center_y + 0.5 * width;
    left_width = 0.01;
    robotArm.addBox('microwave_left', [depth; left_width; height], [center_x; left_y; center_z]);
    
    % right 
    right_y = center_y - 0.5 * width;
    robotArm.addBox('microwave_right', [depth; left_width; height], [center_x; right_y; center_z]);
    
    % upper
    upper_z = center_z + 0.5*height;
    upper_height = 0.01;
    robotArm.addBox('microwave_upper', [depth; width; upper_height], [center_x; center_y; upper_z]);
    
    % lower
    lower_z = center_z - 0.5*height;
    robotArm.addBox('microwave_lower', [depth; width; upper_height], [center_x; center_y; lower_z]);


end


function [ ] = add_fridge(robotArm, H_matrix )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    depth = 0.8;
    width = 0.48;
    height = 0.255;
    % add fridge into the rviz
    
    
    
    % front door
    center_y = H_matrix(2, 4) - 0.03;
    center_z = H_matrix(3, 4) + 0.0225;
    center_x = H_matrix(1, 4) + 0.4;
    %joint_names = robotArm.joint_names;
    %attached_joint = joint_names(7);
    temp_center_x = center_x - 0.5 * depth;
    robotArm.addBox('fridge_front', [0.07; width; height], [temp_center_x; center_y; center_z]);
    robotArm.attachBox('right_gripper', 'fridge_front');
    
    % left
    left_y = center_y + 0.5 * width;
    left_width = 0.01;
    robotArm.addBox('fridge_left', [depth; left_width; height], [center_x; left_y; center_z]);
    
    % right 
    right_y = center_y - 0.5 * width;
    robotArm.addBox('fridge_right', [depth; left_width; height], [center_x; right_y; center_z]);
    
    % upper
    upper_z = center_z + 0.5*height;
    upper_height = 0.01;
    robotArm.addBox('fridge_upper', [depth; width; upper_height], [center_x; center_y; upper_z]);
    
    % lower
    lower_z = center_z - 0.5*height;
    robotArm.addBox('fridge_lower', [depth; width; upper_height], [center_x; center_y; lower_z]);


end


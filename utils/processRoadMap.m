function [ RoadMap, points ] = processRoadMap( RoadMap, arTagposes )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    robotArm = RobotRaconteur.Connect('tcp://localhost:2345/BaxterJointServer/Baxter');
    points = containers.Map;
    for i=1:length(arTagposes.ids)
        right2tag = reshape(arTagposes.tmats((i-1)*16+1: i*16), 4, 4);
        % transformation matrix for the coordinates
        my_rotm = axang2tform([1 0 0 0.5*pi]) * axang2tform([0 1 0 -0.5*pi]);
        base2tag = my_rotm * right2tag;
        % the follow calculation is the orientation of current gripper
        base2tag(1:3, 1:3) = axang2rotm([1 0 0 0.5*pi]) * base2tag(1:3, 1:3);
        base2tag(1:3, 4) = -base2tag(1:3, 4) + robotArm.getPositions('r');
        position = base2tag(1:3,4) + [-0.25; 0.; 0.];
        orientation = rotm2quat(base2tag(1:3,1:3))';
        if arTagposes.ids(i) == 2
            name = 'init_md_ptr';
        elseif arTagposes.ids(i) == 0
            name = 'init_fdd_ptr';
        elseif arTagposes.ids(i) == 5
            name = 'init_fud_ptr';
        end
        point = struct;
        point.name = name;
        point.pos = position;
        point.ori = orientation;
        point.qs = [];
        point.side = '';
        point.AR_id = arTagposes.ids(i);
        points(name) = point;
            
    end
    % only inital point in the roadMap
    if RoadMap.Count == 1
        for key=points.keys
            RoadMap(char(key)) = points(char(key));
        end
    end

end


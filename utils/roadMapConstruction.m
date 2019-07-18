function [ output_args ] = roadMapConstruction( RoadMap, stepSize )
    %UNTITLED Summary of this function goes here
    %   Detailed explanation goes here
    robotArm = RobotRaconteur.Connect('tcp://localhost:2345/BaxterJointServer/Baxter');
    % set initialize point
    init_ptr = RoadMap('init_ptr');
    robotArm.setJointCommand('right', init_ptr.qs);
    index = 1;
    for x = 0:0.05:0.2
        for y = -0.4:stepSize:0.4
            for z = -0.4:stepSize:0.4
                point = RoadMap('init_ptr');
                init_pose = struct;
                init_pose.pos = point.pos;
                init_pose.ori = point.ori;
                end_pose = init_pose;
                end_pose.pos = end_pose.pos + [x; y; z];
                poses = {end_pose};
                frac = robotArm.cartesianPathTraj('right', poses)
                if frac > 0.8
                    name = ['point', num2str(index, '%d')];
                    end_pose.name = name;
                    end_pose.qs = [];
                    RoadMap(char(name)) = end_pose;
                    index = index + 1;
                end
            end
        end
    end
end


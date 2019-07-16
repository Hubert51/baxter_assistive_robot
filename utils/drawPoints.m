function [ output_args ] = drawPoints( RoadMap )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    x = [];
    y = [];
    z = [];
    for key=RoadMap.keys
        point = RoadMap(char(key));
        quiver3(point.pos(1), point.pos(2), point.pos(3), 0, 0.1, 0)
        x = [x, point.pos(1)];
        y = [y, point.pos(2)];
        z = [z, point.pos(3)];    
    end
%     scatter3(y, x, z);
    quiver3(x, y, z, 0, 0.1, 0)
    xlabel('y')
    ylabel('x')
    zlabel('z')
end


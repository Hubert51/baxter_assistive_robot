function [ output_args ] = findFood( Camera, robotArm  )
    %UNTITLED4 Summary of this function goes here
    %   Detailed explanation goes here
    x = 0;
    y = 0;
    img = Camera.getCurrentImage();
    x_low = img.width - 50;
    x_high = img.width + 50;
    y_low = img.height - 50;
    y_high = img.height + 50;

    while x_low < x  && x < x_high && y_low < y && y < y_high
        if x == 0 && y == 0

        else
            pos = robtArm.endeffector_positions;
            if x > x_high
                pos(2) = pos(2) - 0.01; 
            elseif x < x_low
                pos(2) = pos(2) + 0.01;
            end

            if y > y_high
                pos(3) = pos(3) - 0.01;

            elseif y < y_low
                pos(3) = pos(3) + 0.01;
            end
            orientation = robotArm.endeffector_orientations;
            orientation = orientation(1:4) ;
            leftqs = robotArm.solveIKfast(pos, orientation, 'left');
            robotArm.setJointCommand('left', leftqs);        



        imgSub = rossubscriber('/camera/right_hand_camera/image');
        imgMsg = receive( imgSub );
        queryImg = readImage(imgMsg);
    %     imshow(img);
    %     
    %     img = Camera.getCurrentImage();
    %     b = reshape(img.data(1:4:end), img.width, img.height );
    %     g = reshape(img.data(2:4:end), img.width, img.height );
    %     r = reshape(img.data(3:4:end), img.width, img.height );
    %     im = cat(3, r,g,b);
        imshow(im);
        trainImg = imread('query1.jpg');
        trainImg = rgb2gray(trainImg);
        queryImg = rgb2gray(queryImg);
        tform = featureMatcher(queryImg, trainImg);
        x = tform(3,1);
        y = tform(3,2);

        end


    
end


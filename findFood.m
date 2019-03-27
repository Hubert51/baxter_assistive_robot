function [ output_args ] = findFood( Camera  )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
%     imgSub = rossubscriber('/camera/right_hand_camera/image');
%     imgMsg = receive( imgSub );
%     img = readImage(imgMsg);
%     imshow(img);
%     
    img = Camera.getCurrentImage();
    b = reshape(img.data(1:4:end), img.width, img.height );
    g = reshape(img.data(2:4:end), img.width, img.height );
    r = reshape(img.data(3:4:end), img.width, img.height );
    im = cat(3, r,g,b);
    imshow(im);
    trainImg = imread('query1.jpg');
    trainImg = rgb2gray(trainImg);
    queryImg = rgb2gray(im);
    tform = featureMatcher(queryImg, trainImg);
    disp();


    
end


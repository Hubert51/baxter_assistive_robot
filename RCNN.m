% load('rcnnStopSigns.mat', 'stopSigns', 'layers')
% 
% imDir = fullfile(matlabroot, 'toolbox', 'vision', 'visiondata','stopSignImages');
% addpath(imDir);
% 
% layers = [ ...
%     imageInputLayer([32 32 3])
%     convolution2dLayer([5 5],10)
%     reluLayer
%     fullyConnectedLayer(2)
%     softmaxLayer
%     classificationLayer];
% 
% options = trainingOptions('sgdm', ...
%   'MiniBatchSize', 32, ...
%   'InitialLearnRate', 1e-6, ...
%   'MaxEpochs', 10);
% 
% rcnn = trainRCNNObjectDetector(trainingData2, layers, options, 'NegativeOverlapRange', [0 0.3]);
% img = imread('data10081.png');
% img = imread('../test_data/test6.png');

% mode1: resolution is  960 * 600
rosshutdown
rosinit('011303P0004.local', 'NodeHost', '192.168.1.134')
load('count.mat');
while 1
    imgSub = rossubscriber('/cameras/left_hand_camera/image');
    imgMsg = receive(imgSub);
    img = readImage(imgMsg);
    % imshow(img);
    s1 = 'train_data2_baxter_camera/';
    s2 = int2str(count);
    s3 = '.png';
    s = strcat(s1,s2, s3);
    imwrite(img, s)

    [bbox, score, label] = detect(rcnn, img, 'MiniBatchSize', 32);
    [score, idx] = max(score);

    bbox = bbox(idx, :);
    annotation = sprintf('%s: (Confidence = %f)', label(idx), score);

    detectedImg = insertObjectAnnotation(img, 'rectangle', bbox, annotation);
    s1 = 'demo_data/';
    s = strcat(s1,s2, s3);
    imwrite(detectedImg, s)
    count = count + 1;
    save('count.mat', 'count');


    figure
    imshow(detectedImg)
    pause(0.1)
end

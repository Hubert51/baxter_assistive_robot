load('data1.mat')

box1GroundTruth = selectLabels(gTruth,'Box1');
box2GroundTruth = selectLabels(gTruth,'Box2');

% save the images into the files.
% the trainingdata2 stores the file path and the object location
% create a folder here
cd train_data
trainingData2 = objectDetectorTrainingData(box2GroundTruth);
save('summary.mat', trainingData2)
load('summary.mat')
% trainingData1 = objectDetectorTrainingData(box1GroundTruth);

% train the data 
acfDetector = trainACFObjectDetector(trainingData2);

% options = trainingOptions('sgdm', ...
%   'MiniBatchSize', 32, ...
%   'InitialLearnRate', 1e-6, ...
location and %   'MaxEpochs', 10);


img = imread('test_data/test2.png');
img = imread('data10071.png');


[bboxes,scores] = detect(acfDetector,img);
score = max(scores);
for i = 1:length(scores)
    if score ~= scores(i)
        continue
    end
   annotation = sprintf('Confidence = %.3f',scores(i));
   img = insertObjectAnnotation(img,'rectangle',bboxes(i,:),annotation);
end

figure
imshow(img)
function [ tform, theta ] = featureMatcher( query, train )

%% Find Image Rotation and Scale Using Automated Feature Matching
% This example shows how to automatically align two images that differ by a
% rotation and a scale change. It closely parallels another example titled
% <matlab:showdemo('RotationFitgeotransExample') Find Image Rotation and Scale>. 
% Instead of using a manual approach to register the two images, it
% utilizes feature-based techniques found in the Computer Vision System
% Toolbox(TM) to automate the registration process.
%
% In this example, you will use |detectSURFFeatures| and 
% |vision.GeometricTransformEstimator| System object to recover rotation 
% angle and scale factor of a distorted image. You will then transform the 
% distorted image to recover the original image.

% Copyright 1993-2014 The MathWorks, Inc. 

%%
% You can experiment by varying the scale and rotation of the input image.
% However, note that there is a limit to the amount you can vary the scale
% before the feature detector fails to find enough features.

%% Step 3: Find Matching Features Between Images
% Detect features in both images.
ptsQuery  = detectSURFFeatures(query);
ptsTrain = detectSURFFeatures(train);

%%
% Extract feature descriptors.
[featuresQuery,   validPtsQuery]  = extractFeatures(query,  ptsQuery);
[featuresTrain, validPtsTrain]  = extractFeatures(train, ptsTrain);

%%
% Match features by using their descriptors.
indexPairs = matchFeatures(featuresQuery, featuresTrain);

%%
% Retrieve locations of corresponding points for each image.
matchedQuery  = validPtsQuery(indexPairs(:,1));
matchedTrain = validPtsTrain(indexPairs(:,2));

%%
% Show point matches. Notice the presence of outliers.
% figure;
% showMatchedFeatures(original,distorted,matchedQuery,matchedTrain);
% title('Putatively matched points (including outliers)');

%% Step 4: Estimate Transformation
% Find a transformation corresponding to the matching point pairs using the
% statistically robust M-estimator SAmple Consensus (MSAC) algorithm, which
% is a variant of the RANSAC algorithm. It removes outliers while computing
% the transformation matrix. You may see varying results of the
% transformation computation because of the random sampling employed by the
% MSAC algorithm.
[tform, inlierTrain, inlierQuery] = estimateGeometricTransform(...
    matchedTrain, matchedQuery, 'similarity');

%%
% Display matching point pairs used in the computation of the
% transformation matrix.
% figure;
% showMatchedFeatures(original,distorted, inlierQuery, inlierTrain);
% title('Matching points (inliers only)');
% legend('ptsOriginal','ptsDistorted');

%% Step 5: Solve for Scale and Angle
% Use the geometric transform, TFORM, to recover 
% the scale and angle. Since we computed the transformation from the
% distorted to the original image, we need to compute its inverse to 
% recover the distortion.
%
%  Let sc = scale*cos(theta)
%  Let ss = scale*sin(theta)
%
%  Then, Tinv = [sc -ss  0;
%                ss  sc  0;
%                tx  ty  1]
%
%  where tx and ty are x and y translations, respectively.
%

%%
% Compute the inverse transformation matrix.
Tinv  = tform.invert.T;
ss = Tinv(2,1);
sc = Tinv(1,1);
scale_recovered = sqrt(ss*ss + sc*sc);
theta = atan2(ss,sc)*180/pi;

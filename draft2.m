
clear
queryImg = rgb2gray(imread('image/img51.png'));
trainImg = rgb2gray( imread('image/train1.jpg') );
queryImg_copy = queryImg; 



[h, w] = size(queryImg_copy);
[tform, theta] = featureMatcher( queryImg, trainImg );
disp(tform.T)

[tform, theta] = calculate_tform( queryImg, trainImg );




disp(tform.T)

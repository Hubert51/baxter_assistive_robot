% function [ result ] = check_diff_state( old, new, data )
% %UNTITLED2 Summary of this function goes here
% %   Detailed explanation goes here
%     old = 
%     diff = sum((new - old).^2);
%     while
% 
% end
% 
% old = record(:,1) ;
% for i = 1:100
%     new = record(:,i);
%     diff = sum( (new-old).^2 );
%     disp(diff);
%     old = new;
%     pause(0.5);
% end

img = leftCamera.getCurrentImage();
b = reshape(img.data(1:4:end), img.width, img.height );
g = reshape(img.data(2:4:end), img.width, img.height );
r = reshape(img.data(3:4:end), img.width, img.height );
im = cat(3, r,g,b);
im = imrotate(im, 270);
imshow(im)


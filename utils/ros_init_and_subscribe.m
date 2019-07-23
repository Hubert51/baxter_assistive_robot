% initialize the node
rosshutdown
rosinit('011303P0004.local', 'NodeHost', '192.168.1.134')
imgSub = rossubscriber('/cameras/right_hand_camera/image');
imgMsg = receive(imgSub);
img = readImage(imgMsg);
imshow(img);
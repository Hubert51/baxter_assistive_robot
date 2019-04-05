rightCamera = RobotRaconteur.Connect('tcp://localhost:4567/BaxterCameraServer/right_hand_camera');
% leftCamera = RobotRaconteur.Connect('tcp://localhost:1091/BaxterCameraServer/left_hand_camera');
% robotPeripheries = RobotRaconteur.Connect('tcp://localhost:5677/BaxterPeripheralServer/BaxterPeripherals');

% waiting for the initialization of the cameras
rightCamera.openCamera();
% leftCamera.openCamera();
pause(3.5); 

load('cameraparams_right.mat');
% load('/home/jarvis/Baxter_project/right_hand_calib/cameraparams_right.mat');
% load('/home/jarvis/Baxter_project/left_hand_calib/cameraparams_left.mat');
rightCamera.setCameraIntrinsics(cameraparams_right);

leftCamera.setMarkerSize(0.055);

pause(1);

for i = 1: 100
    arTagposes = rightCamera.ARtag_Detection();
    disp(arTagposes)
    pause(2)   
end
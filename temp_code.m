base2foodtag = Hbase2leftcam * left2tag;
position = base2foodtag(1:3,4) - [0.2 ;0.16; 0] ;
orientation = rotm2quat(base2foodtag(1:3,1:3))' - [0;0.1;0;0.1];
leftqs = robotArm.solveIKfast(position, orientation, side);
robotArm.setJointCommand(side, leftqs);






% robotArm.endeffector_positions
% robotArm.endeffector_orientations
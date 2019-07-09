clearvars -except robot gripper
close all

%% Robot Initialization
% robot = RobotRaconteur.Connect('tcp://localhost:4545/BaxterJointServer/Baxter');%robot.setControlMode(uint8(1));
% gripper = RobotRaconteur.Connect('tcp://localhost:46604/BaxterPeripheralServer/BaxterPeripherals');
% gripper.calibrateGripper('left');
% gripper.calibrateGripper('right');
% gripper.suppressBodyAvoidance('left',uint8(1));
% gripper.suppressBodyAvoidance('right',uint8(1));
% gripper.suppressContactSafety('left',uint8(1));
% gripper.suppressContactSafety('right',uint8(1));
% gripper.suppressCollisionAvoidance('left',uint8(1));
% gripper.suppressCollisionAvoidance('right',uint8(1));

[baxter_const, baxter_structure] = defineBaxter();
q0 = [0.3804    0.0579   -1.6341    1.2805    0.1603    1.3948    0.0667]';
q0L = q0;
q0R = q0.*[-1,1,-1,1,-1,1,-1]';
robot.setControlMode(uint8(0));
robot.setJointCommand('left',q0L);
robot.setJointCommand('right',q0R);
% gripper.openGripper('Left');
% gripper.openGripper('Right');

display('Initialization');
pause

%% Process Initialization
% gripper.closeGripper('Left');
% gripper.closeGripper('Right');
robot.setControlMode(uint8(1));

qL = q0L;
qR = q0R;

wr0 = robot.endeffector_wrenches;
pe0 = robot.endeffector_positions;

I6 = 0.0001*eye(6,6);
Ts = 0.02;
[b_filter,a_filter] = butter(2,0.01);
b_fir = fir1(9,0.01); hd = dfilt.dffir(b_fir);

Filter_Buff = 30;
for i =1:Filter_Buff
    wr = robot.endeffector_wrenches-wr0;
    EEWrench(i,:) = [wr]; 
    EEWrench_filter(i,:) = [wr];
    EEWrench_Kalman(i,:) = [wr];
end


%% Process Start
GraspForce = [0,0,0,-4,4,0,0,0,0,-4,-4,0];
move_flag = 0;
K_motion = 0.02;
motion_th = 1.5;
K_com_X = 0.02;
K_com_Y = 0.02;
K_com_Z = 0.02;
K_form = 0.1;

% Kalman Filter
q = 0; 
r = [0.0344    0.0210    0.0907    0.2243    0.0917    0.0959    0.0454    0.0235    0.1346    0.3354    0.1225    0.0922];
Xk_prev = zeros(24,1);
Xk=[];
Phi = [eye(12),Ts*eye(12); zeros(12,12),eye(12)];
P = eye(24,24);
Q = [Ts^3*diag(r.^2),zeros(12,12); zeros(12,12),Ts^3*diag(r.^2)];
M = [eye(12),zeros(12,12)];
R = Ts*diag(r.^2);

for i = 1:2000
    tic;
 
    JLtmp = robotjacobian(baxter_const(1).kin,qL);
    JRtmp = robotjacobian(baxter_const(2).kin,qR);   

    if(cond(JRtmp)<30 && cond(JLtmp)<30)
        JR = JRtmp;   
        JL = JLtmp;   
    else
        display('Out of reach!');
    end

    wr = robot.endeffector_wrenches-wr0;
    EEWrench(i+Filter_Buff,:) = [wr];
    wr_filter = filter(hd,EEWrench); %filter(b_filter,a_filter,EEWrench); %0.05*wr+0.95*wr_filter;
    wr_filter = wr_filter(end,:);
    EEWrench_filter(i+Filter_Buff,:) = [wr_filter];
    % Kalman iteration
    Z = wr;
    P1 = Phi*P*Phi' + Q;
    S = M*P1*M' + R;
    K = P1*M'*inv(S);
    P = P1 - K*M*P1;    
    Xk = Phi*Xk_prev + K*(Z-M*Phi*Xk_prev);
    EEWrench_Kalman(i+Filter_Buff,:) = Xk(1:12);

    % For the next iteration
    Xk_prev = Xk; 
    
    
    V = zeros(12,1);
    
    pe = robot.endeffector_positions;
    
    wr_select = Xk(1:12);
    % X-axis
    V(4) = -K_com_X*(wr_select(4) - GraspForce(4));  
    V(10) = -K_com_X*(wr_select(10) - GraspForce(10));    
    
    % Y-axis
    V(5) = -K_com_Y*(wr_select(5) - GraspForce(5));% - K_form*(pe(2)-pe(5)+(pe0(5)-pe0(2)));  
    V(11) = -K_com_Y*(wr_select(11)- GraspForce(11));% - K_form*(pe(5)-pe(2)+(pe0(2)-pe0(5)));
%     V(1:3) = V(1:3) - K_form*(pe(1:3)-pe(4:6)+(pe0(4:6)-pe0(1:3)));
%     V(4:6) = V(4:6) - K_form*(pe(4:6)-pe(1:3)+(pe0(1:3)-pe0(4:6)));
%     V(6) = -K_com_Z*(wr_select(6) - 0);% - K_form*(pe(2)-pe(5)+(pe0(5)-pe0(2)));  
%     V(12) = -K_com_Z*(wr_select(12)- 0);% - K_form*(pe(5)-pe(2)+(pe0(2)-pe0(5)));

    % Motion Following
    normV(i) = norm(V);
    if (move_flag ==0 & normV(i)<0.015 )
        display('Start moving ...');
        move_flag = 1;
        GraspForce(6) = wr_filter(6);
        GraspForce(12) = wr_filter(12);
    end
    if (move_flag ==1)
%         % X-axis
%         if abs(wr_select(4)-GraspForce(4))> motion_th
%             V(4) = V(4) - K_motion*(wr_select(4)-GraspForce(4));   
%         end
%         if abs(wr_select(10)-GraspForce(10))> motion_th
%             V(10) = V(10) - K_motion*(wr_select(10)-GraspForce(10));    
%         end        
%         % Y-axis
%         if abs(wr_select(5)-GraspForce(5))> motion_th
%             V(5) = V(5) - 1*(wr_select(5)-GraspForce(5));   
%         end
%         if abs(wr_select(11)-GraspForce(11))> motion_th
%             V(11) = V(11) - 1*(wr_select(11)-GraspForce(11));    
%         end  
        % Z-axis
%         if abs(wr_select(6)-GraspForce(6))> 2
%             V(6) = V(6) - K_motion*(wr_select(6)-GraspForce(6));   
%         end
%         if abs(wr_select(12)-GraspForce(12))> 2
%             V(12) = V(12) - K_motion*(wr_select(12)-GraspForce(12));    
%         end  
        detect(i,:)=[(wr_select(6)-EEWrench_Kalman(i+Filter_Buff-1,6)),(wr_select(12)-EEWrench_Kalman(i+Filter_Buff-1,12))];
        
        if abs(wr_select(6)-EEWrench_Kalman(i+Filter_Buff-1,6))> 0.15
            V(6) = V(6) - 0.01*(wr_select(6)-GraspForce(6));%V(6) = V(6) - 1*(wr_select(6)-EEWrench_Kalman(i+Filter_Buff-1,6));   
        end
        if abs(wr_select(12)-EEWrench_Kalman(i+Filter_Buff-1,12))> 0.15
            V(12) = V(12) - 0.01*(wr_select(12)-GraspForce(12)); %V(12) = V(12) - 1*(wr_select(6)-EEWrench_Kalman(i+Filter_Buff-1,12));    
        end          
        
    end

    qLdot = JL'*inv(JL*JL'+I6)*V(1:6);  
    qRdot = JR'*inv(JR*JR'+I6)*V(7:12);
    robot.setJointCommand('left',qLdot*0.5);
    robot.setJointCommand('right',qRdot*0.5);
    V_all(i,:) = V;

    runtimie(i) = toc;
    if (runtimie(i)<Ts)
        pause(Ts-runtimie(i))
        runtimie(i) = toc;
    end
end


robot.setControlMode(uint8(0));
robot.setJointCommand('right',q0R);
robot.setJointCommand('left',q0L);
% gripper.openGripper('Left');
% gripper.openGripper('Right');

figure(1);
subplot(3,1,1); 
plot(EEWrench(:,[4,10])); hold on;
plot(EEWrench_Kalman(:,[4,10]));hold off;
title('X-axis');

subplot(3,1,2); 
plot(EEWrench(:,[5,11])); hold on;
plot(EEWrench_Kalman(:,[5,11]));hold off;
title('Y-axis');

subplot(3,1,3); 
plot(EEWrench(:,[6,12])); hold on;
plot(EEWrench_Kalman(:,[6,12]));hold off;
title('Z-axis');

figure(2);
subplot(3,1,1); 
plot(EEWrench(:,[6,12])); hold on;
plot(EEWrench_Kalman(:,[6,12]));hold off;
title('Z-axis'); xlim([0,2000]);
subplot(3,1,2); 
plot(diff(EEWrench(:,[6,12]))); hold on;
plot(diff(EEWrench_Kalman(:,[6,12])));hold off;
title('Z-axis Diff');xlim([0,2000]);
subplot(3,1,3); 
plot(V_all(:,[6,12])); hold on;
plot(detect);hold off;
title('Z-axis V');xlim([0,2000]);
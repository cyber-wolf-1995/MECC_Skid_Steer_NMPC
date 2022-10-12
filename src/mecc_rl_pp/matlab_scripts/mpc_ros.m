%% Initialize ROS
rosshutdown;clear;clc;close all;
ipaddress = "http://192.168.1.214:11311";
rosinit(ipaddress);

%% Pub, Sub, Controller
amcl_pose = rossubscriber('/amcl_pose','DataFormat','struct');
[vel_pub,msg] = rospublisher("/husky_velocity_controller/cmd_vel", "geometry_msgs/Twist");
imu_data = rossubscriber('/imu/data','DataFormat','struct');
% bag = rosbag('/home/ajoglek/husky_ws/src/huskynick/bags/waypoints_2022-03-16-18-14-25.bag');
% msgs = readMessages(bag);
load updated_rectangle_path.mat;
%path = [path;path;path];
path = circshift(path,50);
path = [path;path];
controller = controllerPurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.5;
controller.MaxAngularVelocity = 0.3;
controller.LookaheadDistance = 0.2;

pose = get_pose(amcl_pose);
robotInitialLocation = pose;
robotGoal = path(end,:);
distanceToGoal = norm(robotInitialLocation - robotGoal);
goalRadius = 0.2;

figure(1)
plot(path(:,1), path(:,2),'k--d');
xlim([-4 9])
ylim([-7 3])
hold all;

mpc_vx = [];
mpc_w = [];
mpc_traj = [];
mpc_wy =[];
control = [];
i = 1;
ct = zeros(1,440);
while (i < 450-11)
   tic
   U = mpc_kinematic(path(i:i+10,:),pose);
   ct(i+1) = toc;
   mpc_vx = [mpc_vx, U(1,1)];
   mpc_w = [mpc_w, U(2,1)];
   %for j = 1:10
   msg.Linear.X = U(1,1);
   msg.Angular.Z = U(2,1);
   send(vel_pub, msg);
   %end
   pose = get_pose(amcl_pose);
   imu = receive(imu_data);
   mpc_wy = [mpc_wy, imu.AngularVelocity.Y];
   %scatter(i,imu.AngularVelocity.Z) 
   mpc_traj = [mpc_traj, pose];
   control = [control; U];
   plot(pose(1),pose(2),'r:s') 
   %distanceToGoal = norm(pose(1:2) - robotGoal(:))
   i = i+1;
end
ct= mean(ct);
% for j=1:10
%     msg.Linear.X = U(1,j);
%     msg.Angular.Z = U(2,j);
%     send(vel_pub, msg);
% end
% msg.Linear.X = 0;
% msg.Angular.Z = 0;
% send(vel_pub, msg);


%% 
% function current_pose = amcl_callback(~,message)
%     global robot_pose
%     x = message.Pose.Pose.Position.X;
%     y = message.Pose.Pose.Position.Y;
%35     robotInitialLocation = [x y];
%     quat = [message.Pose.Pose.Orientation.W message.Pose.Pose.Orientation.X message.Pose.Pose.Orientation.Y message.Pose.Pose.Orientation.Z];
%     eul = quat2eul(quat);
%     initialOrientation = eul(1,1);
%     robot_pose = [robotInitialLocation initialOrientation]';
% end

function pose = get_pose(amcl_pose)
    first_msg = receive(amcl_pose,10);
    x = first_msg.Pose.Pose.Position.X;
    y = first_msg.Pose.Pose.Position.Y;
    robotInitialLocation = [x y];
    quat = [first_msg.Pose.Pose.Orientation.W first_msg.Pose.Pose.Orientation.X first_msg.Pose.Pose.Orientation.Y first_msg.Pose.Pose.Orientation.Z];
    eul = quat2eul(quat);
    initialOrientation = eul(1,1);
    pose = [robotInitialLocation initialOrientation]';
end
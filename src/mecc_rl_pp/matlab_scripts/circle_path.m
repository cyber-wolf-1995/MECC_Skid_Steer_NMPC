rosshutdown;clear;clc;

ipaddress = "http://localhost:11311";
rosinit(ipaddress);

amcl_pose = rossubscriber('/amcl_pose', "DataFormat", "struct");
[vel_pub,vel_msg] = rospublisher("/husky_velocity_controller/cmd_vel", "geometry_msgs/Twist");

speed = 0.2;
radius = 1;

vel_msg.Linear.X = speed;
dist_trav = 0;

rece_msg = receive(amcl_pose);
x = rece_msg.Pose.Pose.Position.X;
y = rece_msg.Pose.Pose.Position.Y;
v0 = [x y];
    
while dist_trav < radius
    send(vel_pub,vel_msg);
    
    new_pose = receive(amcl_pose);
    newx = new_pose.Pose.Pose.Position.X;  
    newy = new_pose.Pose.Pose.Position.Y;  
    newv = [newx newy];
    
    dist_trav = norm(newv-v0);
end
vel_msg.Linear.X = 0;
send(vel_pub, vel_msg);

angular_speed = deg2rad(90);
vel_msg.Angular.Z = angular_speed;
current_angle = 0;
relative_angle = deg2rad(180);

rece_msg = receive(amcl_pose);
Xo = rece_msg.Pose.Pose.Orientation.X;
Yo = rece_msg.Pose.Pose.Orientation.Y;
Zo = rece_msg.Pose.Pose.Orientation.Z;
Wo = rece_msg.Pose.Pose.Orientation.W;
quat0=[Wo Xo Yo Zo];
eul0=quat2eul(quat0);

while current_angle < relative_angle
    send(vel_pub,vel_msg);
    rece_msg = receive(amcl_pose);
    Xo = rece_msg.Pose.Pose.Orientation.X;
    Yo = rece_msg.Pose.Pose.Orientation.Y;
    Zo = rece_msg.Pose.Pose.Orientation.Z;
    Wo = rece_msg.Pose.Pose.Orientation.W;
    quat=[Wo Xo Yo Zo];
    eul=quat2eul(quat);
    current_angle = norm(eul(1,1)-eul0(1,1));
end
vel_msg.Angular.Z = 0;
send(vel_pub, vel_msg);
% 
% dist = 5;
% k=0;
% 
% while k < dist
%     vel_msg.Linear.X = speed;
%     vel_msg.Angular.Z = speed/radius;
%     
% end
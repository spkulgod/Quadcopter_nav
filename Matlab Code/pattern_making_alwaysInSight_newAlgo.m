clear; close all;
camclient  = rossvcclient('/ardrone/setcamchannel');
camreq = rosmessage(camclient);
camreq.Channel=1;
camresp = call(camclient,camreq);
vel = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
vel_msg = rosmessage(vel);
vel_msg.Linear.Y = 0;
vel_msg.Linear.Z = 0;
vel_msg.Linear.X = 0;
send(vel,vel_msg);
takeoff = rospublisher('ardrone/takeoff','std_msgs/Empty');
toff_msg = rosmessage(takeoff);
send(takeoff,toff_msg);
req_altd = 1100;
test = rossubscriber('/ardrone/navdata');
scandata = receive(test,10);
while scandata.Altd<req_altd
    vel_msg.Linear.Z = 0.005*(-scandata.Altd+req_altd);
    
    send(vel,vel_msg);
    scandata = receive(test,10);
end
pause(1)

test = rossubscriber('/ardrone/navdata');
j=1;
image_topic = rossubscriber('/ardrone/image_raw');

format shortg;
start = clock;
X_des=500;
Y_des = 500;
p=0;
X = zeros(1,500);
Y = zeros(1,500);
Vx = zeros(1,500);
Vy = zeros(1,500);
Tm = zeros(1,500);
SizeX = zeros(1,500);
SizeY = zeros(1,500);
DesVx=zeros(1,500);
DesVy=zeros(1,500);
G = zeros(1,500);
G1 = zeros(1,500);
marker = [5,6,7,8,9,10,11,12,13];
X_marker = [-50,0,50,0,0,50,-50,0,50]*10;
Y_marker = [50,50,50,-50,0,0,-50,-50,-50]*10;
%Marker Order
%1 2 3
%4 5 6
%7 8 9

while j<=400
    image_msg = receive(image_topic,2);
    i = readImage(image_msg);
    [rel_pos,marker_num,scale,curr_yaw] = detect_marker_localisation(i);
    if marker_num>0
        pos = rel_pos+[X_marker(marker_num),Y_marker(marker_num)]
        Xpos(j) = pos(1);
        Ypos(j) = pos(2);
        F(j) = im2frame(i);
        scandata = receive(test,10);
        Vx(j) = scandata.Vx;
        Vy(j) = scandata.Vy;
        Tm(j) = scandata.Tm;
        SizeX(j) = 130;
        SizeY(j) = 130;
        alpha = rad2deg(atan2(tan(atan2(pos(2),pos(1)) - pi/2 - atan2(0.1*
        (sqrt(pos(1)^2+pos(2)^2)-300),1)),1));
        error = rad2deg(atan2(tan(deg2rad(curr_yaw-alpha)),1));
        if j>50
            Vel_x = 250;
        else
            Vel_x=0;
        end
        if j>1
            Vel_y=0;
            Roll_Force = -0.0045*(scandata.Vx-Vel_x) + 
                    0.02*(Vx(j-1)-Vx(j));
            Pitch_Force = -0.006*(scandata.Vy-Vel_y) + 
                    0.02*(Vy(j-1)-Vy(j));
            appliedEngineForce = (9.8*.42)/(cos(deg2rad(scandata.RotX))*
                    cos(deg2rad(scandata.RotY)));
            Roll_Force = min(Roll_Force,appliedEngineForce);
            Pitch_Force = min(Pitch_Force,appliedEngineForce);
            
            vel_msg.Linear.X = sign(1.57-acos(Roll_Force/
                    appliedEngineForce))*min(abs(1.57-acos(Roll_Force/
                    appliedEngineForce))/3,0.2);
            vel_msg.Linear.Y = sign(1.57-acos(Pitch_Force/
                    appliedEngineForce))*min(abs(1.57-acos(Pitch_Force/
                    appliedEngineForce))/3,0.2);
            DesVx(j)=Vel_x;
            DesVy(j)=Vel_y;
        end
        
        Error(j)=error;
        if abs(error)>2 && j>1
            vel_msg.Angular.Z = sign(0.1*error+0.05*(Error(j)-Error(j-1)))*
                    min(abs(0.1*error+0.05*(Error(j)-Error(j-1))),0.2);
            send(vel,vel_msg);
            
        else
            vel_msg.Angular.Z = 0;
            send(vel,vel_msg);
        end
        
        ZCom(j) = vel_msg.Angular.Z;
        detect =1
        G(j) = vel_msg.Linear.X;
        j=j+1;
        send(vel,vel_msg);
        p=0;
    else
        land = rospublisher('ardrone/land','std_msgs/Empty');
        land_msg = rosmessage(land)
        send(land,land_msg);
        break;
    end
end
endt = clock - start;
vel_msg.Linear.Y = 0;
vel_msg.Linear.Z = 0;
vel_msg.Linear.X = 0;
send(vel,vel_msg);
pause(1)
land = rospublisher('ardrone/land','std_msgs/Empty');
land_msg = rosmessage(land)
send(land,land_msg);
movie2avi(F,'actual.avi');

subplot(3,2,1);
plot(X-X_des);
subplot(3,2,2);
plot(Y-Y_des);
subplot(3,2,3);
plot(G1);
subplot(3,2,4);
plot(G);
subplot(3,2,5);
hold on;
plot(Vy);
plot(DesVy);
hold off;
subplot(3,2,6);
hold on;
plot(Vx);
plot(DesVx);
hold off;


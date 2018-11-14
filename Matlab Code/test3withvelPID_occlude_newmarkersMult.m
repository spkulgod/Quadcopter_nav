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
X_marker = [0,90,180,0,90,180,0,90,180]*10;
Y_marker = [200,200,200,100,100,0,0,0,100]*10;
%Marker Order
%1 2 3
%4 5 6
% %7 8 9
waypoints = [7,2,6,3,1,7,5,6];
size_wp = size(waypoints,2);
k=1;
q=0;
num_circ = marker(waypoints(1))
while k<=size_wp
    if q==70
        q=0;
        k=k+1;
        if k>size_wp
            break
        end
        num_circ=marker(waypoints(k));
        Xc = 500+(X_marker(waypoints(k))-X_marker(waypoints(k-1)))*(130/200);
        Yc = 500-(Y_marker(waypoints(k))-Y_marker(waypoints(k-1)))*(130/200);
    end
    image_msg = receive(image_topic,2);
    i = readImage(image_msg);
    [center,curr_yaw] = calc_center(num_circ,i)
    F(j) = im2frame(i);
    scandata = receive(test,10);
    if (size(center,1))
        q=q+1;
        p=0;
        Xc = double(center(1))*100/64;
        Yc = double(center(2))*100/32;
        X(j)=Xc;
        Y(j)=Yc;
        Vx(j) = scandata.Vx;
        Vy(j) = scandata.Vy;
        Tm(j) = scandata.Tm;
        SizeX(j) = 130;
        SizeY(j) = 130;
        if j>1
            
            Vel_x = -1*(Yc-Y_des) + 0.8*(Y(j-1)-Y(j));
            Vel_y= -1*(Xc-X_des) + 0.8*(X(j-1)-X(j));
            Roll_Force = -0.007*(scandata.Vx-Vel_x) + 0.025*(Vx(j-1)-Vx(j));
            Pitch_Force = -0.007*(scandata.Vy-Vel_y) + 0.026*(Vy(j-1)-Vy(j));
            appliedEngineForce = (9.8*.42)/(cos(deg2rad(scandata.RotX))*cos(deg2rad(scandata.RotY)));
            Roll_Force = min(Roll_Force,appliedEngineForce);
            Pitch_Force = min(Pitch_Force,appliedEngineForce);
            
            vel_msg.Linear.X = sign(1.57-acos(Roll_Force/appliedEngineForce))*min(abs(1.57-acos(Roll_Force/appliedEngineForce))/3,0.3);
            vel_msg.Linear.Y = sign(1.57-acos(Pitch_Force/appliedEngineForce))*min(abs(1.57-acos(Pitch_Force/appliedEngineForce))/3,0.3);
            
            DesVx(j)=Vel_x;
            DesVy(j)=Vel_y;
            PosX(j) = (Yc-Y_des)*200/130 + Y_marker(waypoints(k));
            PosY(j) = -(Xc-X_des)*200/130 + X_marker(waypoints(k));
        end
        detect =1
        G(j) = vel_msg.Linear.X;
        G1(j) = vel_msg.Linear.Y;
        scandata = receive(test,1);
        j=j+1;
    else
        if j>2 && p==0
            j=j-1;
        end
        if p<100 && j>2
            curr_t = scandata.Tm;
            if ~p
            else
                vbot = [0,0];
                Yc = Yc -(vbot(1) - scandata.Vx*double(SizeX(j))/200)*(curr_t-prev_t)*1e-6;
                Xc = Xc -(vbot(2) - scandata.Vy*double(SizeY(j))/200)*(curr_t-prev_t)*1e-6;
                j=j+1;
                X(j)=Xc;
                Y(j)=Yc;
                Vx(j) = scandata.Vx;
                Vy(j) = scandata.Vy;
                SizeX(j) = SizeX(j-1);
                SizeY(j) = SizeY(j-1);
                Tm(j) = scandata.Tm;
                Vel_x = -1.2*(Yc-Y_des) + 0.4*(Y(j-1)-Y(j));
                Vel_y= -1.2*(Xc-X_des) + 0.4*(X(j-1)-X(j));
                Roll_Force = -0.007*(scandata.Vx-Vel_x) + 0.025*(Vx(j-1)-Vx(j));
                Pitch_Force = -0.007*(scandata.Vy-Vel_y) + 0.02*(Vy(j-1)-Vy(j));
                appliedEngineForce = (9.8*.42)/(cos(deg2rad(scandata.RotX))*cos(deg2rad(scandata.RotY)));
                Roll_Force = min(Roll_Force,appliedEngineForce);
                Pitch_Force = min(Pitch_Force,appliedEngineForce);
                vel_msg.Linear.X = sign(1.57-acos(Roll_Force/appliedEngineForce))*min(abs(1.57-acos(Roll_Force/appliedEngineForce))/3,0.1);
                vel_msg.Linear.Y = sign(1.57-acos(Pitch_Force/appliedEngineForce))*min(abs(1.57-acos(Pitch_Force/appliedEngineForce))/3,0.1);
                send(vel,vel_msg);
                G(j) = vel_msg.Linear.X;
                G1(j) = vel_msg.Linear.Y;
                DesVx(j)=Vel_x;
                DesVy(j)=Vel_y;
                PosX(j) = (Yc-Y_des)*200/130 + Y_marker(waypoints(k));
                PosY(j) = -(Xc-X_des)*200/130 + X_marker(waypoints(k));
            end
            
            p = p+1
            prev_t = curr_t;
            
        end
        if p==100
            break
        end
        scandata = receive(test,1);
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
plot(X_des-X);
title('Error Y-Position(in Pixels)');
xlabel('Number of iterations');
ylabel('Error');
subplot(3,2,2);
plot(Y_des-Y);
title('Error X-Position(in Pixels)');
xlabel('Number of iterations');
ylabel('Error');
subplot(3,2,3);
plot(G1);
title('Roll command');
xlabel('Number of iterations');
ylabel('Roll command');
subplot(3,2,4);
plot(G);
title('Pitch command');
xlabel('Number of iterations');
ylabel('Pitch command');
subplot(3,2,5);
hold on;
plot(Vy);
plot(DesVy);
hold off;
title('Actual and Desired Y-velocity');
xlabel('Number of iterations');
ylabel('Actual and Desired Y-velocity');
subplot(3,2,6);
hold on;
plot(Vx);
plot(DesVx);
hold off;
title('Actual and Desired X-velocity');
xlabel('Number of iterations');
ylabel('Actual and Desired X-velocity');

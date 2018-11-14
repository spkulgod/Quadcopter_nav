% Program to land the quadcopter on to a moving platform

clear

%Initial Setting
camclient  = rossvcclient('/ardrone/setcamchannel');
camreq = rosmessage(camclient);
camreq.Channel=1;
camresp = call(camclient,camreq);
odom = rossubscriber('/ardrone/odometry');
vel = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
vel_msg = rosmessage(vel);
vel_msg.Linear.Y = 0;
vel_msg.Linear.X = 0;
send(vel,vel_msg);

%Takeoff
takeoff = rospublisher('ardrone/takeoff','std_msgs/Empty');
toff_msg = rosmessage(takeoff);
send(takeoff,toff_msg);

%Check if the quad has stabilised
test = rossubscriber('/ardrone/navdata');
scandata = receive(test,1);
scandata.State
while (scandata.State~=4)
    test = rossubscriber('/ardrone/navdata');
    scandata = receive(test,1);
    scandata.State
    scandata.Altd
end

%Change altitude
req_altd = 1000;
while scandata.Altd<req_altd
    vel_msg.Linear.Z = 0.005*(-scandata.Altd+req_altd);
    send(vel,vel_msg);
    scandata = receive(test,10);
end
vel_msg.Linear.Z = 0;
send(vel,vel_msg);


start = clock
image_topic = rossubscriber('/ardrone/image_raw');
for j=1:250
    %Receive image and find out the distance of the centre of the tag from
    %The centre of the quadcopter
    image_msg = receive(image_topic,2);
    i = readImage(image_msg);
    F(j)=im2frame(i);
    
    %While tag is in view
    while (size(scandata.TagsXc,1))
        Vel_msg.Linear.X = -0.08*(scandata.TagsXc-500)/500;
        vel_msg.Linear.Y = -0.08*(scandata.TagsYc-500)/500;
        send(vel,vel_msg)
        detect =1
        scandata = receive(test,1)
        if ~(size(scandata.TagsXc,1))
            land = rospublisher('ardrone/land','std_msgs/Empty')
            land_msg = rosmessage(land)
            send(land,land_msg);
            break;
        end
    end
    
    %While tag is not in view
    filt = i(:,:,1)<10 & i(:,:,2)<10 & i(:,:,3)<10;
    props = regionprops( double(filt), 'Centroid' );
    if size(props,1)
        cent = props.Centroid(1);
    elseif j>1
        cent = F2(j-1);
    else
        cent = 320
    end
    F2(j)=cent;
    if (cent>340 || cent<300) && (j>1)
        F3(j) = -0.08*(cent-320)/300 + 0.005*(F2(j-1)-F2(j));
        vel_msg.Linear.Y = F3(j);
        send(vel,vel_msg);
    else
        F3(j) = 0;
        vel_msg.Linear.Y = F3(j);
        vel_msg.Linear.X = .05;
        send(vel,vel_msg);
    end
        
    %output odometer data
    odomdata = receive(odom,1)
end
pause(1)

%land the quadcopter
land = rospublisher('ardrone/land','std_msgs/Empty')
land_msg = rosmessage(land)
send(land,land_msg);

%calculating the time elapsed
land1 = clock - start

%convert the image clips to a avi format video
movie2avi(F,'actual.avi');

%check battery percentage
battery = scandata.BatteryPercent
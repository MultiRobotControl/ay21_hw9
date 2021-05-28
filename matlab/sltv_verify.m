%% Load .bag file and create bag file object. 


<<<<<<< HEAD
fname = '2021-05-27-23-28-22.bag'; 
=======
<<<<<<< HEAD
fname = '2021-05-26-21-12-54.bag'; 
=======
fname = '2021-05-23-20-48-36.bag'; 
>>>>>>> d50c9c38303ba30ef937bdbf5b4826a23cc001e8
>>>>>>> 88137ca1300eea6aacf4e6ee17da7efbbf6f7d21
bag = rosbag(fname);
 
% Display available topics and message types in bag file. 
bag.AvailableTopics

% Create a time series of the Odometry data
% Retrieve the messages as a cell array
odom1_msgs = select(bag,'Topic','/cora1/cora/sensors/p3d');
 
% Create a timeseries object of the subset of message fields we are interested in
odom1_ts = timeseries(odom1_msgs,'Pose.Pose.Position.X','Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y',...
    'Pose.Pose.Orientation.Z','Twist.Twist.Linear.X','Twist.Twist.Angular.Z');

odom2_msgs = select(bag,'Topic','/cora2/cora/sensors/p3d');
 
% Create a timeseries object of the subset of message fields we are interested in
odom2_ts = timeseries(odom2_msgs,'Pose.Pose.Position.X','Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y',...
    'Pose.Pose.Orientation.Z','Twist.Twist.Linear.X','Twist.Twist.Angular.Z');

% Create a time series of the Cmd data
% Retrieve the messages as a cell array
cmd1_msgs = select(bag,'Topic','/cora1/cora/cmd_vel');
 
cmd1_ts = timeseries(cmd1_msgs,'Linear.X','Linear.Y','Linear.Z',...
        'Angular.X','Angular.Y','Angular.Z');
    
cmd2_msgs = select(bag,'Topic','/cora2/cora/cmd_vel');
 
cmd2_ts = timeseries(cmd2_msgs,'Linear.X','Linear.Y','Linear.Z',...
        'Angular.X','Angular.Y','Angular.Z');

% Create a time series of the Odometry data
% Retrieve the messages as a cell array
rabbit_msgs = select(bag,'Topic','/rabbit');
 
% Create a timeseries object of the subset of message fields we are interested in
rabbit_ts = timeseries(rabbit_msgs,'Point.X','Point.Y','Point.Z');
<<<<<<< HEAD
%% 1) Plot Distance of each USV with Virtual Leader
X0 = rabbit_ts.Data(1:1128,1);
Y0 = rabbit_ts.Data(1:1128,2);
X1 = odom1_ts.Data(1:1128,1);
Y1 = odom1_ts.Data(1:1128,2);
X2 = odom2_ts.Data(:,1);
Y2 = odom2_ts.Data(:,2);

dist1 = sqrt((X1-X0).^2 + (Y1-Y0).^2);  
dist2 = sqrt((X2-X0).^2 + (Y2-Y0).^2); 
figure(1); clf();
% Plot the Data index corresponding to 'Pose.Pose.Position.X' & 'Pose.Pose.Position.Y'
plot(odom1_ts.Time(1:1128),dist1,'b','LineWidth', 1); hold on
plot(odom2_ts.Time,dist2,'r','LineWidth', 2); 
title('(1) Distance Between Each USV and Virtual Leader')
legend('USV1 Position','USV2 Position', 'Location','best')
xlabel('Time')
ylabel('Distance')
grid on

%% 2) Plots of Distance between USVs

Udist = sqrt((X2-X1).^2 + (Y2-Y1).^2);
figure(2); clf();
% Plot the Data index corresponding to 'Pose.Pose.Position.X' & 'Pose.Pose.Position.Y'
plot(odom1_ts.Time(1:1128), Udist,'b','LineWidth',1);
title('Inter-Vehicle Distance Between USV1 and USV2')
xlabel('Time')
ylabel('Distance')
=======
%% 1) Plot X-Y Position of USV

figure(1); clf();
% Plot the Data index corresponding to 'Pose.Pose.Position.X' & 'Pose.Pose.Position.Y'
plot(odom1_ts.Data(:,1),odom1_ts.Data(:,2),'b','LineWidth', 1); hold on
plot(odom2_ts.Data(:,1),odom2_ts.Data(:,2),'r','LineWidth', 2); 
plot(rabbit_ts.Data(:,1),rabbit_ts.Data(:,2),'k-.'); 
title('(1) X-Y Position of USVs and Rabbit')
legend('USV1 Position','USV2 Position', 'Rabbit Position', 'Location','best')
xlabel('X')
ylabel('Y')
grid on

%% 2) Plots of X vs time and Y vs time of USV

figure(2); clf();
% Plot the Data index corresponding to 'Pose.Pose.Position.X' & 'Pose.Pose.Position.Y'
subplot(2,1,1)
plot(odom1_ts.Time, odom1_ts.Data(:,1),'b','LineWidth',1); hold on
plot(odom2_ts.Time, odom2_ts.Data(:,1),'r','LineWidth',2); 
plot(rabbit_ts.Time, rabbit_ts.Data(:,1),'k-.');
title('X Position vs Time')
legend('USV1 Position','USV2 Position', 'Rabbit Position', 'Location','best')
xlabel('Time')
ylabel('X')
grid on

subplot(2,1,2)
plot(odom1_ts.Time,odom1_ts.Data(:,2),'b','LineWidth',1); hold on
plot(odom2_ts.Time,odom2_ts.Data(:,2),'r','LineWidth',2); 
plot(rabbit_ts.Time, rabbit_ts.Data(:,2),'k-.');
title('Y Position vs Time')
legend('USV1 Position','USV2 POsition', 'Rabbit Position', 'Location','best')
xlabel('Time')
ylabel('Y')
>>>>>>> 88137ca1300eea6aacf4e6ee17da7efbbf6f7d21
grid on
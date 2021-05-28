%% Load .bag file and create bag file object. 


fname = '2021-05-27-23-28-22.bag'; 
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
grid on
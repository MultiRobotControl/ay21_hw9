clear

% Globals
% We will declare global variables that this function and the callbacks
% can all access
% When we receive an odometry messge, save it here.
global USV1_ODOM;  
global USV2_ODOM; 
global USV3_ODOM; 

% When we receive a rabbit posiition, save it here
global RABBIT_POSITION;

% Try to start ROS - if it is already started, restart
try
    rosinit
catch
    rosshutdown
    rosinit
end

% Subscribers
usv1_sub = rossubscriber('/cora1/cora/sensors/p3d',@usv1_odom_callback, ...
    'DataFormat', 'struct');
usv2_sub = rossubscriber('/cora2/cora/sensors/p3d',@usv2_odom_callback, ...
    'DataFormat', 'struct');
usv3_sub = rossubscriber('/cora3/cora/sensors/p3d',@usv3_odom_callback, ...
    'DataFormat', 'struct');

% Add another subscriber here for the rabbit!
rabbit_sub = rossubscriber('/rabbit',@rabbit_callback, ...
    'DataFormat', 'struct');

% For now we'll just assign a blank message
RABBIT_POSITION = rosmessage('geometry_msgs/PointStamped');

% Setup Publisher
cmd1_pub = rospublisher('/cora1/cora/cmd_vel','geometry_msgs/Twist');
cmd1_msg = rosmessage(cmd1_pub);
cmd2_pub = rospublisher('/cora2/cora/cmd_vel','geometry_msgs/Twist');
cmd2_msg = rosmessage(cmd2_pub);
cmd3_pub = rospublisher('/cora3/cora/cmd_vel','geometry_msgs/Twist');
cmd3_msg = rosmessage(cmd3_pub);

% Infinite loop
while true
    
    if isempty(RABBIT_POSITION)
        disp('Warning - Rabbit Position is empty.')
        v1_c=0;r1_c=0;v2_c=0;r2_c=0;v3_c=0;r3_c=0; 
    elseif isempty(USV1_ODOM)
        disp('Warning - USV Odometry is empty.')
        v1_c=0;r1_c=0; v2_c=0;r2_c=0;v3_c=0;r3_c=0;  
    else
        % Call a function to implement the VBAP algorithm.
        [v1_c,r1_c,v2_c,r2_c,v3_c,r3_c] = vbap_multi3(USV1_ODOM, USV2_ODOM,USV3_ODOM, RABBIT_POSITION);

        % Publish the results
        cmd1_msg.Linear.X = v1_c;
        cmd1_msg.Angular.Z = r1_c;
        send(cmd1_pub, cmd1_msg);
        
        % Publish the results
        cmd2_msg.Linear.X = v2_c;
        cmd2_msg.Angular.Z = r2_c;
        send(cmd2_pub, cmd2_msg);
        
        % Publish the results
        cmd3_msg.Linear.X = v3_c;
        cmd3_msg.Angular.Z = r3_c;
        send(cmd3_pub, cmd3_msg);
        
        pause(0.1);
    end 
end
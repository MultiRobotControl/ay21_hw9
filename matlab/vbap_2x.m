clear

% Globals
% We will declare global variables that this function and the callbacks
% can all access
% When we receive an odometry messge, save it here.
global USV1_ODOM;  
global OTHER_USV_ODOMS; 
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

% Infinite loop
while true
    
    if isempty(RABBIT_POSITION)
        disp('Warning - Rabbit Position is empty.')
<<<<<<< HEAD
        v_c=0;r_c=0; 
    elseif isempty(USV1_ODOM)
        disp('Warning - USV Odometry is empty.')
        v_c=0;r_c=0; 
    else
        % Call a function to implement the VBAP algorithm.
        [v_c, r_c] = vbap_multi(USV1_ODOM, OTHER_USV_ODOMS, RABBIT_POSITION);
        
        % Publish the results
        cmd1_msg.Linear.X = v_c(1);
        cmd1_msg.Angular.Z = r_c(1);
        send(cmd1_pub, cmd1_msg);
        
        % Publish the results
        cmd2_msg.Linear.X = v_c(2);
        cmd2_msg.Angular.Z = r_c(2);
        send(cmd2_pub, cmd2_msg);
        
        fprintf('v1_c=%.2f, r1_c=%.2f,v2_c=%.2f, r2_c=%.2f',v_c(1),r_c(1),v_c(2),r_c(2)); 
        
=======
        v1_c=0;r1_c=0;v2_c=0;r2_c=0; 
    elseif isempty(USV1_ODOM)
        disp('Warning - USV Odometry is empty.')
        v1_c=0;r1_c=0;v2_c=0;r2_c=0; 
    else
        % Call a function to implement the VBAP algorithm.
        [v1_c, r1_c,v2_c,r2_c] = vbap_sltv(USV1_ODOM, OTHER_USV_ODOMS, RABBIT_POSITION);
        
        % Publish the results
        cmd1_msg.Linear.X = v1_c;
        cmd1_msg.Angular.Z = r1_c;
        send(cmd1_pub, cmd1_msg);
        
        % Publish the results
        cmd2_msg.Linear.X = v2_c;
        cmd2_msg.Angular.Z = r2_c;
        send(cmd2_pub, cmd2_msg);
        
>>>>>>> d50c9c38303ba30ef937bdbf5b4826a23cc001e8
        pause(0.1);
    end 
end
function usv2_odom_callback(~, msg)

% Example callback function to be called with odometry message

% For testing only - print a message when this function is called.
%disp('Received USV2 Odometry')

% Declare global variables to store odometry message
global OTHER_USV_ODOMS;

OTHER_USV_ODOMS = msg;

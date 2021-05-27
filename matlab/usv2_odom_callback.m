function usv2_odom_callback(~, msg)

% Example callback function to be called with odometry message

% For testing only - print a message when this function is called.
%disp('Received USV2 Odometry')

% Declare global variables to store odometry message
<<<<<<< HEAD
global OTHER_USV_ODOMS;

OTHER_USV_ODOMS = msg;
=======
global USV2_ODOM;

USV2_ODOM = msg;
>>>>>>> d50c9c38303ba30ef937bdbf5b4826a23cc001e8

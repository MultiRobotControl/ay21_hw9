<<<<<<< HEAD

=======
>>>>>>> 88137ca1300eea6aacf4e6ee17da7efbbf6f7d21
function usv1_odom_callback(~, msg)

% Example callback function to be called with odometry message

% For testing only - print a message when this function is called.
%disp('Received USV1 Odometry')

% Declare global variables to store odometry message
global USV1_ODOM;

USV1_ODOM = msg;

function [u_c, r_c] = vbap_slsv(USV_ODOM, RABBIT_POSITION)
% Function prototype for implementing 
% Virtual Body, Artificial Potential - Single Leader, Single Vehicle

dx = RABBIT_POSITION.Point.X - USV_ODOM.Pose.Pose.Position.X;
dy = RABBIT_POSITION.Point.Y - USV_ODOM.Pose.Pose.Position.Y;

quat = USV_ODOM.Pose.Pose.Orientation; 
angles = quat2eul([quat.W quat.X quat.Y quat.Z]); 
psi = angles(:,1);

dist = sqrt(dx^2 + dy^2);
theta = atan2(dy,dx);

aerr = wrapToPi(theta-psi);

kv=0.1;
kh=2.0;
u_c = kv*dist;
r_c = kh*aerr;

return
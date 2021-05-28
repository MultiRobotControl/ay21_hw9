function [v1_c, r1_c,v2_c,r2_c] = vbap_multi2(USV1_ODOM,OTHER_USV_ODOMS, RABBIT_POSITION)
% Function prototype for implementing 
% Potential 
d0 = 25;
d1 = 2*d0;

% Distance with Rabbit
Xerr1 = RABBIT_POSITION.Point.X - USV1_ODOM.Pose.Pose.Position.X;
Yerr1 = RABBIT_POSITION.Point.Y - USV1_ODOM.Pose.Pose.Position.Y;
Xerr2 = RABBIT_POSITION.Point.X - OTHER_USV_ODOMS.Pose.Pose.Position.X;
Yerr2 = RABBIT_POSITION.Point.Y - OTHER_USV_ODOMS.Pose.Pose.Position.Y;

% Distance with Other USVs
Xuerr1 = USV1_ODOM.Pose.Pose.Position.X - OTHER_USV_ODOMS.Pose.Pose.Position.X;
Yuerr1 = USV1_ODOM.Pose.Pose.Position.Y - OTHER_USV_ODOMS.Pose.Pose.Position.Y;

% Convert to quaternions
quat1 = USV1_ODOM.Pose.Pose.Orientation; 
angles1 = quat2eul([quat1.W quat1.X quat1.Y quat1.Z]); 
psi1 = angles1(1);
quat2 = OTHER_USV_ODOMS.Pose.Pose.Orientation; 
angles2 = quat2eul([quat2.W quat2.X quat2.Y quat2.Z]); 
psi2 = angles2(1);

%Desired heading
teta1 = atan2(Yerr1,Xerr1);
teta2 = atan2(Yerr2,Xerr2);

%Errors
distErr1 = sqrt(Xerr1^2 + Yerr1^2); 
headErr1 = wrapToPi(teta1-psi1); 
distErr2 = sqrt(Xerr2^2 + Yerr2^2); 
headErr2 = wrapToPi(teta2-psi2); 
kv = 0.1; kh = 3; k0 = 0.05; 

hij = sqrt(Xuerr1^2 + Yuerr1^2); 
eij = k0 * (hij - d0); 

if hij <= d1
    Ur1_c = eij * sign(atan2(YuErr1,XuErr1) - psi1); 
else 
    Ur1_c = 0; 
end 

v1_c = kv*distErr1; 
v2_c = kv*distErr2;
r1_c = kh * headErr1; 
r2_c = kh * headErr2;

fprintf("USV1--> Teta=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    teta1,psi1,headErr1,r1_c,distErr1,v1_c);
fprintf("USV2--> Teta=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    teta2,psi2,headErr2,r2_c,distErr2,v2_c);
fprintf("USV1 to 2--> Teta=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    teta2,psi2,headErr2,r2_c,distErr2,v2_c);

return

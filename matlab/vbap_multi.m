function [v_c, r_c] = vbap_multi(USV1_ODOM,OTHER_USV_ODOMS, RABBIT_POSITION)
% Function prototype for implementing 
% Potential 
d0 = 25;
d1 = 2*d0;
kv = 0.1; kh = 1; k0 = 0.05; 

% Loop to make column vector of distance errors and teta errors
for i=1:height(OTHER_USV_ODOMS)
    % Distance with Rabbit
    Xerr(1) = RABBIT_POSITION.Point.X - USV1_ODOM.Pose.Pose.Position.X;
    Yerr(1) = RABBIT_POSITION.Point.Y - USV1_ODOM.Pose.Pose.Position.Y; 
    Xerr(i+1) = RABBIT_POSITION.Point.X - OTHER_USV_ODOMS{i}.Pose.Pose.Position.X;
    Yerr(i+1) = RABBIT_POSITION.Point.Y - OTHER_USV_ODOMS{i}.Pose.Pose.Position.Y;
    XUerr(i) = OTHER_USV_ODOMS{i}.Pose.Pose.Position.X - USV1_ODOM.Pose.Pose.Position.X;
    YUerr(i) = OTHER_USV_ODOMS{i}.Pose.Pose.Position.Y - USV1_ODOM.Pose.Pose.Position.Y; 
    psiLead = atan2(Yerr,Xerr);
    psiIJ = atan2(YUerr,XUerr); 
    
    quat = USV1_ODOM.Pose.Pose.Orientation; 
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]); 
    psi(1) = angles(1);
    
    quat2 = OTHER_USV_ODOMS{i}.Pose.Pose.Orientation; 
    angles2 = quat2eul([quat2.W quat2.X quat2.Y quat2.Z]); 
    psi(i+1) = angles2(1);
end 

psiL = kh * wrapToPi(psiLead - psi);

hIJ = sqrt(Xerr^2 + Yerr^2); 
eIJ = k0 * (hij - d0); 

psiJ = eIJ * sign(wrapToPi(psiIJ - psi)); 

if hij <= d1
    psiJ = eIJ * sign(atan2(YuErr,XuErr) - psi); 
else 
    psiJ = 0; 
end 

v_c = kv*distErr1; 
r_c = kh * headErr1; 

fprintf("USV1--> Teta=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    teta1,psi1,headErr1,r1_c,distErr1,v1_c);
fprintf("USV2--> Teta=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    teta2,psi2,headErr2,r2_c,distErr2,v2_c);
fprintf("USV1 to 2--> Teta=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    teta2,psi2,headErr2,r2_c,distErr2,v2_c);

return

function [v_c, r_c] = vbap_multi(USV1_ODOM,OTHER_USV_ODOMS, RABBIT_POSITION)
% Function prototype for implementing 
% Potential 
d0 = 25;
d1 = 2*d0;
kv = 1; kh = 1; k0 = 0.1; 

% Loop to make column vector of distance errors and teta errors
for i=1:height(OTHER_USV_ODOMS)
    % Distance with Rabbit
    Xerr(1) = RABBIT_POSITION.Point.X - USV1_ODOM.Pose.Pose.Position.X;
    Yerr(1) = RABBIT_POSITION.Point.Y - USV1_ODOM.Pose.Pose.Position.Y; 
    Xerr(i+1) = RABBIT_POSITION.Point.X - OTHER_USV_ODOMS(i).Pose.Pose.Position.X;
    Yerr(i+1) = RABBIT_POSITION.Point.Y - OTHER_USV_ODOMS(i).Pose.Pose.Position.Y;
    
    XUerr(i) = OTHER_USV_ODOMS(i).Pose.Pose.Position.X - USV1_ODOM.Pose.Pose.Position.X;
    YUerr(i) = OTHER_USV_ODOMS(i).Pose.Pose.Position.Y - USV1_ODOM.Pose.Pose.Position.Y; 
    psiLead = atan2(Yerr,Xerr);
    psiIJ(1) = 0; % No command from self
    psiIJ(i+1) = atan2(YUerr(i),XUerr(i)); 
    
    quat = USV1_ODOM.Pose.Pose.Orientation; 
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]); 
    psi(1) = angles(1);
    
    quat2 = OTHER_USV_ODOMS(i).Pose.Pose.Orientation; 
    angles2 = quat2eul([quat2.W quat2.X quat2.Y quat2.Z]); 
    psi(i+1) = angles2(1);
end 

psiL = kh * wrapToPi(psiLead - psi);

distErr = sqrt(Xerr.^2 + Yerr.^2); 
hIJ = sqrt(XUerr.^2 + YUerr.^2);
eIJ = k0 * (hIJ - d0); 

if hIJ < d1
    psiJ = eIJ * sign(wrapToPi(psiIJ - psi)); 
else 
    psiJ = 0; 
end 

v_c = kv*distErr;
r_c = psiL + sum(psiJ);

fprintf("distErr=%.2f, hIJ=%.2f \n",...
    distErr,hIJ);

return

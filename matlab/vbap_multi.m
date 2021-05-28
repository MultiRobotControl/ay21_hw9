<<<<<<< HEAD
function [v1_c, r1_c,v2_c,r2_c] = vbap_multi(USV1_ODOM,USV2_ODOM, RABBIT_POSITION)
% Function prototype for implementing 
kv = 0.1; kh = 1; k0 = 0.1;
d0 = 25; d1 = 2*d0; 

% Distance errors with rabbit
Xerr1 = RABBIT_POSITION.Point.X - USV1_ODOM.Pose.Pose.Position.X;
Yerr1 = RABBIT_POSITION.Point.Y - USV1_ODOM.Pose.Pose.Position.Y;
Xerr2 = RABBIT_POSITION.Point.X - USV2_ODOM.Pose.Pose.Position.X;
Yerr2 = RABBIT_POSITION.Point.Y - USV2_ODOM.Pose.Pose.Position.Y;
% Distance errors with each other 
XUerr1 = USV2_ODOM.Pose.Pose.Position.X - USV1_ODOM.Pose.Pose.Position.X;
YUerr1 = USV2_ODOM.Pose.Pose.Position.Y - USV1_ODOM.Pose.Pose.Position.Y; 

% Convert to quaternions
quat1 = USV1_ODOM.Pose.Pose.Orientation; 
angles1 = quat2eul([quat1.W quat1.X quat1.Y quat1.Z]); 
psi1 = angles1(1);
quat2 = USV2_ODOM.Pose.Pose.Orientation; 
angles2 = quat2eul([quat2.W quat2.X quat2.Y quat2.Z]); 
psi2 = angles2(1);

% Desired heading towards leader
psiLead1 = atan2(Yerr1,Xerr1);
psiLead2 = atan2(Yerr2,Xerr2);

% Distance and heading errors between individual USV and rabbit 
distErr1 = sqrt(Xerr1^2 + Yerr1^2); 
headErr1 = wrapToPi(psiLead1 - psi1); 
distErr2 = sqrt(Xerr2^2 + Yerr2^2); 
headErr2 = wrapToPi(psiLead2 - psi2); 

% Spring force
hIJ = sqrt(XUerr1.^2 + YUerr1.^2);
eIJ = k0 * (hIJ - d0); 
psiIJ = atan2(YUerr1, XUerr1); 
UheadErr1 = wrapToPi(psiIJ - psi1); % psiI = psi1 
UdistErr1 = hIJ - d0; 

if hIJ < d1
    psiJ = eIJ * sign(UheadErr1); 
else 
    psiJ = 0;
end

% Control commands
v1_c = kv * distErr1;
r1_c = kh * headErr1 + sum(psiJ); 
v2_c = kv * distErr2;
r2_c = kh * headErr2;
fprintf("USV1--> PsiLead=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    psiLead1,psi1,headErr1,r1_c,distErr1,v1_c);
fprintf("USV2--> PsiLead=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    psiLead2,psi2,headErr2,r2_c,distErr2,v2_c);
fprintf("USV1 to USV2 --> PsiJ=%.2f, Heading Err=%.2f, Distance Err (hIJ-d0)=%.2f \n", ...
    psiJ,UheadErr1,UdistErr1);

return
=======
function [v_c, r_c] = vbap_multi(USV1_ODOM,OTHER_USV_ODOMS, RABBIT_POSITION)
% Function prototype for implementing 
% Potential 
d0 = 25;
d1 = 2*d0;
<<<<<<< HEAD
kv = 1; kh = 1; k0 = 0.1; 
=======
kv = 0.1; kh = 1; k0 = 0.05; 
>>>>>>> d50c9c38303ba30ef937bdbf5b4826a23cc001e8

% Loop to make column vector of distance errors and teta errors
for i=1:height(OTHER_USV_ODOMS)
    % Distance with Rabbit
    Xerr(1) = RABBIT_POSITION.Point.X - USV1_ODOM.Pose.Pose.Position.X;
    Yerr(1) = RABBIT_POSITION.Point.Y - USV1_ODOM.Pose.Pose.Position.Y; 
<<<<<<< HEAD
    Xerr(i+1) = RABBIT_POSITION.Point.X - OTHER_USV_ODOMS(i).Pose.Pose.Position.X;
    Yerr(i+1) = RABBIT_POSITION.Point.Y - OTHER_USV_ODOMS(i).Pose.Pose.Position.Y;
    XUerr(i) = OTHER_USV_ODOMS(i).Pose.Pose.Position.X - USV1_ODOM.Pose.Pose.Position.X;
    YUerr(i) = OTHER_USV_ODOMS(i).Pose.Pose.Position.Y - USV1_ODOM.Pose.Pose.Position.Y; 
    psiLead = atan2(Yerr,Xerr);
    psiIJ(1) = 0; % No command from self
    psiIJ(i+1) = atan2(YUerr(i),XUerr(i)); 
=======
    Xerr(i+1) = RABBIT_POSITION.Point.X - OTHER_USV_ODOMS{i}.Pose.Pose.Position.X;
    Yerr(i+1) = RABBIT_POSITION.Point.Y - OTHER_USV_ODOMS{i}.Pose.Pose.Position.Y;
    XUerr(i) = OTHER_USV_ODOMS{i}.Pose.Pose.Position.X - USV1_ODOM.Pose.Pose.Position.X;
    YUerr(i) = OTHER_USV_ODOMS{i}.Pose.Pose.Position.Y - USV1_ODOM.Pose.Pose.Position.Y; 
    psiLead = atan2(Yerr,Xerr);
    psiIJ = atan2(YUerr,XUerr); 
>>>>>>> d50c9c38303ba30ef937bdbf5b4826a23cc001e8
    
    quat = USV1_ODOM.Pose.Pose.Orientation; 
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]); 
    psi(1) = angles(1);
    
<<<<<<< HEAD
    quat2 = OTHER_USV_ODOMS(i).Pose.Pose.Orientation; 
=======
    quat2 = OTHER_USV_ODOMS{i}.Pose.Pose.Orientation; 
>>>>>>> d50c9c38303ba30ef937bdbf5b4826a23cc001e8
    angles2 = quat2eul([quat2.W quat2.X quat2.Y quat2.Z]); 
    psi(i+1) = angles2(1);
end 

psiL = kh * wrapToPi(psiLead - psi);

<<<<<<< HEAD
distErr = sqrt(Xerr.^2 + Yerr.^2); 
hIJ = sqrt(XUerr.^2 + YUerr.^2);
eIJ = k0 * (hIJ - d0); 

if hIJ < d1
    psiJ = eIJ * sign(wrapToPi(psiIJ - psi)); 
=======
hIJ = sqrt(Xerr^2 + Yerr^2); 
eIJ = k0 * (hij - d0); 

psiJ = eIJ * sign(wrapToPi(psiIJ - psi)); 

if hij <= d1
    psiJ = eIJ * sign(atan2(YuErr,XuErr) - psi); 
>>>>>>> d50c9c38303ba30ef937bdbf5b4826a23cc001e8
else 
    psiJ = 0; 
end 

<<<<<<< HEAD
v_c = kv*distErr;
r_c = kh * (psiL + sum(psiJ));

fprintf("distErr=%.2f, hIJ=%.2f \n",...
    distErr,hIJ);
=======
v_c = kv*distErr1; 
r_c = kh * headErr1; 

fprintf("USV1--> Teta=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    teta1,psi1,headErr1,r1_c,distErr1,v1_c);
fprintf("USV2--> Teta=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    teta2,psi2,headErr2,r2_c,distErr2,v2_c);
fprintf("USV1 to 2--> Teta=%.2f, Psi=%.2f, Heading Err=%.2f, r_c=%.2f, Distance Err=%.2f, u_c=%.2f\n", ...
    teta2,psi2,headErr2,r2_c,distErr2,v2_c);
>>>>>>> d50c9c38303ba30ef937bdbf5b4826a23cc001e8

return
>>>>>>> 88137ca1300eea6aacf4e6ee17da7efbbf6f7d21

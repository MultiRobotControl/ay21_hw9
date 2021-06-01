function [u_c, r_c] = vbap_multi(USV_ODOM, OTHER_ODOM, RABBIT_POSITION)

for i=1:height(OTHER_ODOM)
    dx(1) = RABBIT_POSITION.Point.X - USV_ODOM.Pose.Pose.Position.X;
    dy(1) = RABBIT_POSITION.Point.Y - USV_ODOM.Pose.Pose.Position.Y;
    dx(i+1) = RABBIT_POSITION.Point.X - OTHER_ODOM(i).Pose.Pose.Position.X;
    dy(i+1) = RABBIT_POSITION.Point.Y - OTHER_ODOM(i).Pose.Pose.Position.Y;

    dx_cora(i) = OTHER_ODOM(i).Pose.Pose.Position.X - USV_ODOM.Pose.Pose.Position.X;
    dy_cora(i) = OTHER_ODOM(i).Pose.Pose.Position.Y - USV_ODOM.Pose.Pose.Position.Y;
    psi_L = atan2(dy_cora,dx_cora);
    psi_ij(1) = 0;
    psi_ij(i+1) = atan2(dy_cora(i),dx_cora(i));
    
    quat1 = USV_ODOM.Pose.Pose.Orientation; 
    angles1 = quat2eul([quat1.W quat1.X quat1.Y quat1.Z]); 
    psi(1) = angles1(1);

    quat2 = OTHER_ODOM(i).Pose.Pose.Orientation; 
    angles2 = quat2eul([quat2.W quat2.X quat2.Y quat2.Z]); 
    psi(i+1) = angles2(1);
end

k_v = 1.0; k_h = 1.0; k_o = 0.1;
d_0 = 25; d_1 = 2 * d_0;

dist = sqrt(dx.^2 + dy.^2);

h_ij = sqrt(dx_cora.^2 + dy_cora.^2);
e_ij = k_o .* (h_ij-d_0);
psi_lead = k_h .* wrapToPi(psi_L - psi);

if h_ij <= d_1
    psiJ = e_ij * sign(wrapToPi(psi_ij - psi));
else
    psiJ = 0;
end

% Total Control Law
u_c = k_v .* dist;
r_c = psi_lead + sum(psiJ);

% Saturate
u_c = min(abs(u_c),10.0);
r_c = min(r_c, 2*pi);
r_c = max(r_c, -2*pi);
return
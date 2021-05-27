function [u_c1,u_c2,r_c1,r_c2] = vbap_multi(USV_ODOM, USV2_ODOM, RABBIT_POSITION)

dx_1 = RABBIT_POSITION.Point.X - USV_ODOM.Pose.Pose.Position.X;
dy_1 = RABBIT_POSITION.Point.Y - USV_ODOM.Pose.Pose.Position.Y;
dx_2 = RABBIT_POSITION.Point.X - USV2_ODOM.Pose.Pose.Position.X;
dy_2 = RABBIT_POSITION.Point.Y - USV2_ODOM.Pose.Pose.Position.Y;

dist1 = sqrt(dx_1^2 + dy_1^2);
dist2 = sqrt(dx_2^2 + dy_2^2);

quat1 = USV_ODOM.Pose.Pose.Orientation; 
angles1 = quat2eul([quat1.W quat1.X quat1.Y quat1.Z]); 
psi1 = angles1(:,1);
aerr1 = wrapToPi(atan2(dy_1,dx_1)-psi1);

quat2 = USV2_ODOM.Pose.Pose.Orientation; 
angles2 = quat2eul([quat2.W quat2.X quat2.Y quat2.Z]); 
psi2 = angles2(:,1);
aerr2 = wrapToPi(atan2(dy_2,dx_2)-psi2);

k_v = 0.5; k_h = 1.0; k_o = 0.25;
d_0 = 25; d_1 = 2 * d_0;

dx_cora = USV_ODOM.Pose.Pose.Position.X - USV2_ODOM.Pose.Pose.Position.X;
dy_cora = USV_ODOM.Pose.Pose.Position.Y - USV2_ODOM.Pose.Pose.Position.Y;

h_2 = sqrt(dx_cora^2 + dy_cora^2);
e_2 = k_o * (h_2-d_0);

if h_2 <= d_1
    r_2 = e_2 * sign(atan2(dy_cora,dx_cora) - psi1);
else
    r_2 = 0;
end




% Total Control Law
u_c1 = k_v * dist1;
u_c2 = k_v * dist2;
r_c1 = k_h * aerr1;
r_c2 = k_h * (aerr2 + r_2);

% Saturate
u_c1 = min(abs(u_c1),0.0);
u_c2 = min(abs(u_c2),0.0);
u_c1 = max(abs(u_c1),10.0);
u_c2 = max(abs(u_c2),10.0);
r_c1 = min(r_c1, 2*pi);
r_c1 = max(r_c1, -2*pi);
r_c2 = min(r_c2, 2*pi);
r_c2 = max(r_c2, -2*pi);
return
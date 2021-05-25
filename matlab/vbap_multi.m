function [u_c, r_c] = vbap_multi(USV_ODOM, USV2_ODOM, RABBIT_POSITION)

dx = USV_ODOM.Pose.Pose.Position.X - USV2_ODOM.Pose.Pose.Position.X;
dy = USV_ODOM.Pose.Pose.Position.Y - USV2_ODOM.Pose.Pose.Position.y;

psi = atan2(dy,dx);

dist_cora = (sqrt(dx^2 + dy^2)-25) * sign(psi);

k_o = 0.5;
k_h = 1.0;

u_c = k_o * dist_cora;
r_c = k_h * psi;

return
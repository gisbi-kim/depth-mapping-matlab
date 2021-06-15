function [scan_global] = local2global(pc, lidar_to_base_init_se3, scan_pose_se3)

xyz = pc.Location;

scan_hmg = [xyz, ones(length(xyz), 1)]';
scan_global_hmg = scan_pose_se3 * lidar_to_base_init_se3 * scan_hmg; % remember this order (left multiplication!)
scan_global = scan_global_hmg(1:3, :)';
scan_global = pointCloud(scan_global);

scan_global.Color = pc.Color;

end


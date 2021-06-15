function [ points ] = depthImg2Ptcloud(depthImg, intrinsic, rgbImg)

depth_scale = 1000; % mm

fx = intrinsic(1, 1);
fy = intrinsic(2, 2); 
cx = intrinsic(1, 3);
cy = intrinsic(2, 3);

% ref https://kr.mathworks.com/matlabcentral/answers/456165-point-cloud-from-depth-and-rgb-image
depth = double(depthImg);

Sd = size(depth);
[X,Y] = meshgrid(1:Sd(2),1:Sd(1));

X = X - cx + 0.5;
Y = Y - cy + 0.5;
XDf = depth / fx;
YDf = depth / fy;
X = X .* XDf;
Y = Y .* YDf;

XY = cat(3, X, Y);
cloud = cat(3, XY, depth);
cloud = reshape(cloud, [], 3) / depth_scale;
colors = reshape(rgbImg, [], 3);
cloud = pointCloud(cloud);
cloud.Color = colors;

points = cloud;

end


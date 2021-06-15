% 
clear; clc;

scanGlobalMapInited = 0;
figure(3); clf;

%% setup 

% rgb cam (720x960) intrinsic 
cam_intrinsic = ...
[ 7.9600665283203125e+02, 0., 4.7114190673828125e+02; ...
  0., 7.9600665283203125e+02, 3.5450680541992188e+02; ...
  0., 0., 1];

% load data info 
rgbImgDir = "/home/user/Documents/RTAB-Map/stairs/imgs/rgb/";
rgbImgNames = listdir(rgbImgDir);

depthImgDir = "/home/user/Documents/RTAB-Map/stairs/imgs/depth/";
depthImgNames = listdir(depthImgDir);

posesDir = "/home/user/Documents/RTAB-Map/stairs/poses/";
posesPath = fullfile(posesDir, "poses_robot.txt");

poses = readmatrix(posesPath);


%% main 
for ii=1:length(depthImgNames)
    
    %% load data 
    rgbImgName = rgbImgNames{ii};
    rgbImgPath = fullfile(rgbImgDir, rgbImgName);
    rgbImg = imread(rgbImgPath);
    rgbImgDownsized = imresize(rgbImg, 192/720);

    depthImgName = depthImgNames{ii};
    depthImgPath = fullfile(depthImgDir, depthImgName);
    depthImg = imread(depthImgPath);
    depthImgUpsampled = imresize(depthImg, 720/192, 'nearest'); 
        % note that do not interpolate (to prevent noisy depths)

    poseLine = poses(ii, :);
    poseSE3 = [reshape(poseLine, 4, 3)'; 0,0,0,1];

    %% visualization 
    figure(22); clf;

    subplot(3, 1, 1);
    imagesc(rgbImg);
    axis equal; 

    subplot(3, 1, 2);
    imagesc(depthImgUpsampled);
    axis equal; 
    caxis([0, 5000]); % up to 5 meter coloring 
    colormap jet;

    % 3d point cloud using depth image 
    subplot(3, 1, 3);
%     scanLocal = depthImg2PtcloudV0(depthImgUpsampled, cam_intrinsic, rgbImg); 
    scanLocal = depthImg2Ptcloud(depthImgUpsampled, cam_intrinsic, rgbImg); 
    % here, we assume the depth and rgb camera shares the same HW (same
    % intrinsic for the same size of an image) - because RTAB-Map export
    % does not tell us depth image intrinsics 
    % thus, use the upsampled depth image 
    
    scanGlobal = local2global(scanLocal, eye(4), (poseSE3));
    scanGlobal = pcdownsample(scanGlobal, 'gridAverage', 0.01); 
        % to remove repeated or null points 
    pcshow(scanGlobal); 
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    colormap jet;
    view(-180, 90);
    set(gcf,'color','w');

    %% global map 
    figure(3); clf;
    if( ~ scanGlobalMapInited)
        scanGlobalMap = scanGlobal;
        scanGlobalMapInited = 1;
    else
        scanGlobalMap = pcmerge(scanGlobalMap, scanGlobal, 0.01);  % to remove repeated points 
    end
    pcshow(scanGlobalMap, 'MarkerSize', 30); hold on; 
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)');
    colormap jet;
    view(-180, 90);
    set(gcf,'color','w');
    
    % 
    pause(0.01);
end



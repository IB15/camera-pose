%% Overview
% This example shows how to extract 3D geometry from hand-held unsteady 
% uncalibrated cameras and how to reconstruct a scene from multiple 2-D
% images. This type of reconstruction can be used to develop 3-D models of
% objects or build a world maps.  The steps of a reconstruction process
% are shown and explained using a pair of images.

clc;
clear all;
close all;

%% Intrinsic parameters

% Load a pair of images into the workspace.
uiwait(msgbox('Choose two photos for the estimation of the intrinsic camera parameters','modal'));
I1e = imread (uigetfile('*.jpg'));
I2e = imread (uigetfile('*.jpg'));
imshowpair(I1e, I2e, 'montage'); 
title('Pair of Original Images');

% Find Homogrphy and estimate K
[H] = sift_ransacmatch(I1e, I2e);
[K, f_length] = estimate_K_ax(H);
f_length
cameraParams = cameraParameters('IntrinsicMatrix',K);

%% Camera Pose Estimation

uiwait(msgbox('Choose two photos for which you want to estimate the camera pose','modal'));
I1 = imread (uigetfile('*.jpg'));
I2 = imread (uigetfile('*.jpg'));
figure;
imshowpair(I1, I2, 'montage'); 
title('Pair of Original Images');

% Add info
choice = questdlg('Do you know the real dimensions of an object in the scene?', ...
    'Info');
switch choice
    case 'Yes'
        disp([choice ' -> 3D Pose Estimation'])
        flag = 1;
    case 'No'
        disp([choice ' -> 2D Pose Estimation'])
        flag = 0;
    case 'Cancel'
        disp([choice ' -> Exiting...'])
        return
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              3D Plot                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3D PLOT
% Calculate Extrinsics and Build Camera Matrices for Images
% First you need to detect 4 points in the image, and define the corresponding world points. 
% Then use the |extrinsics| function to compute the rotation and the translation of the
% camera relative to the points selected. This rotation and translation, 
% together with the intrinsic parameters are stored in the Camera Matrix. 
% The matrix relates 3-D world positions to 2-D pixel locations in the images.  

if (flag==1)

% WORLD POINTS
worldPoints = [0,0;0,0;0,0;0,0;];
              
% IMAGE refpoints1 
figure; 
imshow(I1);
hold on;
refPoints1 = zeros( size(worldPoints) );
for k = 1 : 4
    p = GetPoint('r+');
    refPoints1(k,1:2) = p;
    text(p(1)+30,p(2)-50,num2str(k), 'Color', 'r');
end
hold off;

% IMAGE  refpoints2
figure; 
imshow(I2);
hold on;
refPoints2 = zeros( size(worldPoints) );
for k = 1 : 4
    p = GetPoint('r+');
    refPoints2(k,1:2) = p;
    text(p(1)+30,p(2)-50,num2str(k), 'Color', 'r');
end
hold off;

% Give the world reference
prompt = 'Insert the distance between point ''1'' and point ''2'' in the real world (mm): ';
worldPoints(2,1) = input(prompt);
prompt = 'Insert the distance between point ''2'' and point ''3'' in the real world (mm): ';
worldPoints(3,2) = input(prompt);
prompt = 'Insert the distance between point ''3'' and point ''4'' in the real world (mm): ';
worldPoints(3,1) = input(prompt);
prompt = 'Insert the distance between point ''4'' and point ''1'' in the real world (mm): ';
worldPoints(4,2) = input(prompt);

[R1, t1] = extrinsics(refPoints1, worldPoints, cameraParams);
[R2, t2] = extrinsics(refPoints2, worldPoints, cameraParams);

% Calculate camera matrices using the |cameraMatrix| function.
cameraMatrix1 = cameraMatrix(cameraParams, R1, t1);
cameraMatrix2 = cameraMatrix(cameraParams, R2, t2);

%% Extract Feature Points 
% You need to find a set of points common to both images in order to
% calculate their corresponding 3-D locations.  Detect and extract point
% feature descriptors.  

% Detect feature points
imagePoints1 = detectSURFFeatures(rgb2gray(I1), 'MetricThreshold', 600);
imagePoints2 = detectSURFFeatures(rgb2gray(I2), 'MetricThreshold', 600);

% Extract feature descriptors
features1 = extractFeatures(rgb2gray(I1), imagePoints1);
features2 = extractFeatures(rgb2gray(I2), imagePoints2);

% Visualize several extracted SURF features from the image1
figure;
imshow(I1);
title('1500 Strongest Feature Points from image1');
hold on;
plot(selectStrongest(imagePoints1, 1500));

%% Match Features Across Images
% These features are matched between the images using the matchFeatures
% function.
indexPairs = matchFeatures(features1, features2, 'MaxRatio', 0.4);
matchedPoints1 = imagePoints1(indexPairs(:, 1));
matchedPoints2 = imagePoints2(indexPairs(:, 2));

% Visualize correspondences
figure;
showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2,'montage','PlotOptions',{'ro','go','y--'});
title('Original Matched Features from image1 and image2');

%% Reconstruct the Object From Corresponding Features
% Estimate 3-D locations corresponding to the matched points using the 
% |triangulate| function, which implements the Direct Linear Transformation
% (DLT) algorithm. [1] The function also returns reprojection errors, which
% are the differences between the image points and the reprojected world
% points. These errors can be used to eliminate spurious matches.
 
[points3D, reprojErrors] = triangulate(matchedPoints1, matchedPoints2, ...
    cameraMatrix1, cameraMatrix2);
worldPoints3D = triangulate(refPoints1, refPoints2, ...
    cameraMatrix1, cameraMatrix2);

% Eliminate noisy points
errorDists = max(sqrt(sum(reprojErrors .^ 2, 2)), [], 3);
T = abs(t1(1)-t2(1));
validIdx = errorDists < max(T*0.25,1);

points3D = points3D(validIdx, :);
points3D = [points3D;worldPoints3D];

validPoints1 = matchedPoints1(validIdx, :);
validPoints2 = matchedPoints2(validIdx, :);
validPoints1 = [validPoints1.Location;refPoints1];
validPoints2 = [validPoints2.Location;refPoints2];

figure;
showMatchedFeatures(I1, I2, validPoints1, validPoints2,'montage','PlotOptions',{'ro','go','y--'});
title('Matched Features After Removing Noisy Matches');

%% Draw the 3-D Point Cloud, and the cameras position
% The 3-D points can now be plotted with the same units as the calibration.
% Using the scatter3 function, draw a point cloud representation of each
% feature location and its corresponding colors.

% Draw camera positions and orientations
figure;
plotCamera('Location', -t1 * R1', 'Orientation', R1', 'Size', 50, ...
    'Color', 'r', 'Label', '1');
hold on
grid on
plotCamera('Location', -t2 * R2', 'Orientation', R2', 'Size', 50, ...
    'Color', 'b', 'Label', '2');

% Get the color of each reconstructed point
validPoints1 = round(validPoints1);
numPixels = size(I1, 1) * size(I1, 2);
allColors = reshape(im2double(I1), [numPixels, 3]);
colorIdx = sub2ind([size(I1, 1), size(I1, 2)], validPoints1(:,2), ...
    validPoints1(:, 1));
color = allColors(colorIdx, :);

% Add green point representing the origin, and red points representing the
% choosen spots
points3D(end+1,:) = [0,0,0];
color(end-3,:) = [1,0,0];
color(end-2,:) = [1,0,0];
color(end-1,:) = [1,0,0];
color(end,:)   = [1,0,0];
color(end+1,:) = [0,1,0];

% Plot point cloud
showPointCloud(points3D, color, 'VerticalAxisDir', 'down', 'MarkerSize', 45);
xlabel('x-axis (mm)');
ylabel('y-axis (mm)');
zlabel('z-axis (mm)');
title('Estimated 3D Cameras Position');

hold off;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              2D Plot                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (flag==0)
% Detect feature points
imagePoints1 = detectSURFFeatures(rgb2gray(I1), 'MetricThreshold', 600);
imagePoints2 = detectSURFFeatures(rgb2gray(I2), 'MetricThreshold', 600);

% Extract feature descriptors
features1 = extractFeatures(rgb2gray(I1), imagePoints1);
features2 = extractFeatures(rgb2gray(I2), imagePoints2);

% These features are matched between the images using the matchFeatures
% function.
indexPairs = matchFeatures(features1, features2, 'MaxRatio', 0.3);
matchedPoints1 = imagePoints1(indexPairs(:, 1));
matchedPoints2 = imagePoints2(indexPairs(:, 2));

% Visualize correspondences
figure;
showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2,'montage','PlotOptions',{'ro','go','y--'});
title('Putative point matches');

%compute foundamental
[F, inliers] = estimateFundamentalMatrix(matchedPoints1,matchedPoints2, 'NumTrials', 2000);

matchedPoints1=matchedPoints1(inliers,:);
matchedPoints2=matchedPoints2(inliers,:);

figure;
showMatchedFeatures(I1, I2, matchedPoints1,matchedPoints2,'montage','PlotOptions',{'ro','go','y--'});
title('Point matches after outliers were removed');

%% Find Essential matrix
% Assume the world coordinate is aligned with camera coordinate of the
% first camera

E = K' * F * K;
 
% Find P2
% P2 is find from decompositing the Essential matrix
% From decompositing, the result are 4 possible solution
% From 4 possible solution, find the true solution
P4 = get4possibleP(E);  

% Get Correct P matrix   
m1 = matchedPoints1.Location';
m2 = matchedPoints2.Location';
X_TestPoint = [m1(:,4),m2(:,4)]; X_TestPoint(3,:) = 1; 
P1 = [eye(3) zeros(3,1)];  
P2 = getCorrectCameraMatrix(P4, K, K, X_TestPoint) ;

%% Draw 2D scene
figure;
hold on;
grid on;
xlim([-2 7]);
ylim([-2 2]);

% Plot camera
[K1, R1, C1] = decomposecamera(K*P1);
[K2, R2, C2] = decomposecamera(K*P2);
plot(C1(1,1), C1(2,1), 'ob');
plot(C2(1,1), C2(2,1), 'or');

% Plot direction
[angz1, angy1, angx1] = DecompRMat(R1);
[angz2, angy2, angx2] = DecompRMat(R2);

if (dot(cross(R2(:,1), R2(:,2)), R2(:,3)) < 0) && (C2(2,1) < 0)
    angy2 = -angy2;
end

c = 0.5;
x = [0 C2(1,1);c*cosd(angy1) (C2(1,1)+ c*cosd(angy2))];
y = [0 C2(2,1);c*sind(angy1) (C2(2,1)+ c*sind(angy2))];
plot (x, y);
title('Estimated 2D Cameras Position');

hold off;
end

%% References
% [1] Hartley, Richard, and Andrew Zisserman. Multiple View Geometry in
% Computer Vision. Second Edition. Cambridge, 2000.

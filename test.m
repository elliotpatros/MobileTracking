%% reset
clear all;
addpath(genpath('.'));
load('CameraParameters');

%% plot world points
plot3(worldPoints(:,1), worldPoints(:,2), zeros(size(worldPoints, 1), 1), '*');
hold on;
plot3(0, 0, 0, 'ro');

%% process images
for n = 1:10
    
%% find camera extrinsics
file = ['./Resources/sm_checkerboard-', num2str(n), '.jpg'];
frame = imread(file);
frame = rgb2gray(frame);
frame = undistortImage(frame, cameraParameters);
imagePoints = detectCheckerboardPoints(frame);
[rotation, translation] = extrinsics(imagePoints, worldPoints, cameraParameters);

%% find camera pose
orientation = rotation';
location = -translation/rotation;

%% plot camera
plotCamera('Location', location, 'Orientation', orientation, 'Opacity', 0, 'Size', 10);
end

%% viewing preferences
grid on;
axis equal;
axis([-50, 120, -60 80, -150, 0]);
view(0, -80);%view(20, -65); %
set(gca, 'CameraUpVector', [0 -1 0]);
hold off;

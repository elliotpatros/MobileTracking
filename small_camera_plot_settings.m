%% reset
% clear all;
figure
addpath(genpath('.'));

squareSize = 8; % world units
worldUnits = 'mm';

%% calibrate camera
imageFileNames = getImageFileNames('./Resources/vsm_checkerboard', 10, 'jpg');
    
% detect checkerboards in images
disp('detecting checkerboard points...');
[imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames);
nImagePoints = length(imagePoints);

% get world coordinates of the corners
disp('finding world coordinates...');
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% do calibration
disp('estimating camera parameters...');
cameraParameters = estimateCameraParameters(...
    imagePoints, worldPoints,...
    'EstimateSkew', false, ...
    'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, ...
    'WorldUnits', worldUnits, ...
    'InitialIntrinsicMatrix', [], ...
    'InitialRadialDistortion', []);

%% plot world points
plot3(worldPoints(:,1), worldPoints(:,2), zeros(size(worldPoints, 1), 1), '*');
hold on;
plot3(0, 0, 0, 'ro');

%% process images
for n = 1:10
    
%% find camera extrinsics
frame = imread(imageFileNames{n});
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
view(0, -80); %view(20, -65); %
set(gca, 'CameraUpVector', [0 -1 0]);
hold off;
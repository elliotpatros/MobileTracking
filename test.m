%% reset
clear all;
% figure
addpath(genpath('.'));
% load('CameraParameters');

squareSize = 8; % world units (its actually 20)
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


nSteps = 100;
a = linspace(0, 1, nSteps);
for r = 1:nSteps
    
%% make corrective rotation matrix (location)
X = makehgtform('xrotate', a(r) * 1*pi);
X = X(1:3,1:3);
% Y = makehgtform('yrotate', a(r) * 1*pi);
% Y = Y(1:3,1:3);
% Z = makehgtform('zrotate', a(r) * 1*pi);
% Z = Z(1:3,1:3);

%% plot world points
plot3(worldPoints(:,1), worldPoints(:,2), zeros(size(worldPoints, 1), 1), '*');
hold on;
plot3(0, 0, 0, 'ro');

%% process images
% for n = 1:1
    n = 1;
%% find camera extrinsics
frame = imread(imageFileNames{n});
frame = rgb2gray(frame);
frame = undistortImage(frame, cameraParameters);
imagePoints = detectCheckerboardPoints(frame);
[rotation, translation] = extrinsics(imagePoints, worldPoints, cameraParameters);

%% find camera pose
orientation = rotation';
location = -translation/rotation;

%% correct pose
V = rotationMatrixToVector(orientation);
Y = makehgtform('yrotate', 2 * a(r) * V(2));
Y = Y(1:3,1:3);
Z = makehgtform('zrotate', 2 * a(r) * V(3));
Z = Z(1:3,1:3);

location = location*X;
orientation = X*Y*orientation;
% location(3) = -location(3);

% location = location*Y;
% orientation = Y*orientation;
% location = location*Z;
% orientation = Z*orientation;


% %% make corrective rotation matrix (orientation)
% V = rotationMatrixToVector(orientation);
% R = makehgtform('xrotate', V(1));
% R = R(1:3,1:3);
% 
% orientation = R*orientation;

%% plot camera
plotCamera('Location', location, 'Orientation', orientation, 'Opacity', 0, 'Size', 10);
% end

%% viewing preferences
grid on;
axis equal;
% axis vis3d
% axis([-50, 120, -60 80, -150, 0]);
view(-45, -65); %view(0, -80); %
set(gca, 'CameraUpVector', [0 -1 0]);
hold off;
drawnow
end
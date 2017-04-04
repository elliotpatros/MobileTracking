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

%% make corrective rotation matrix (location)
X = makehgtform('xrotate', pi);
X = X(1:3,1:3);

%% plot world points
sx = -[worldPoints(1,1), worldPoints(end,1), worldPoints(1,1), worldPoints(end,1)];
sy = [worldPoints(1,2), worldPoints(1,2), worldPoints(end,2), worldPoints(end,2)];
sz = zeros(4);
surf(sx, sy, sz);
hold on;
plot3(0, 0, 0, 'ro');

%% process images
for n = 1:10
% find camera extrinsics
frame = imread(imageFileNames{n});
frame = rgb2gray(frame);
frame = undistortImage(frame, cameraParameters);
imagePoints = detectCheckerboardPoints(frame);
[rotation, translation] = extrinsics(imagePoints, worldPoints, cameraParameters);

% find camera pose
orientation = rotation';
location = -translation/rotation;

% correct pose
V = rotationMatrixToVector(orientation);
R = makehgtform('zrotate', -2 * V(3));
R = R(1:3,1:3);

location = location*X;
orientation = X*R*orientation;
location(1) = -location(1);

% plot camera
plotCamera('Location', location, 'Orientation', orientation, 'Opacity', 0.25, 'AxesVisible', true, 'Size', 5);
end

%% viewing preferences
grid on;
axis equal
axis vis3d
axis([-120, 60, -60, 80, 0, 140]);
view(-170, 35);
hold off;
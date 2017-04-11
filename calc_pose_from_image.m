%% reset
clear all;
addpath(genpath('.'));

%% user parameters
ip_addr = '192.168.0.12';
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

%% setup plotting constants
sx = [worldPoints(1,1), worldPoints(end,1), worldPoints(1,1), worldPoints(end,1)];
sy = [worldPoints(1,2), worldPoints(1,2), worldPoints(end,2), worldPoints(end,2)];
sz = zeros(4);

%% get current frame from video stream
try
    frame = rgb2gray(imread('./Resources/vsm_checkerboard-1.jpg'));
catch
    disp('stream closed');
end

subplot(131);
imshow(frame);

%% plot world points
subplot(1,3,[2,3]);
surf(sx, sy, sz);
hold on;
plot3(0, 0, 0, 'ro');

%% process image
% find camera extrinsics
frame = undistortImage(frame, cameraParameters);
imagePoints = detectCheckerboardPoints(frame);

% if we didn't find a checkerboard, continue
if (length(imagePoints) == nImagePoints)
    [rotation, translation] = extrinsics(imagePoints, worldPoints, cameraParameters);

    % find camera pose
    orientation = rotation';
    location = -translation/rotation;

    % plot camera
   
    plotCamera('Location', location, 'Orientation', orientation, 'Opacity', 0.25, 'AxesVisible', true, 'Size', 5);

    %% viewing preferences
    grid on;
    axis equal
    axis vis3d
    axis([-120, 120, -120, 80, -300, 0]);
    view(-10, -80);

    hold off;
end
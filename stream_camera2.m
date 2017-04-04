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
    
%% set up stream
url = ['http://', ip_addr, ':8080/shot.jpg'];
is_streaming = true;

%% stream
disp('streaming...');
while(is_streaming)
    %% get current frame from video stream
    try
        frame = rgb2gray(imread(url));
    catch
        is_streaming = false;
        disp('stream closed');
        break;
    end
    
    %% plot world points
    surf(sx, sy, sz);
    hold on;
    plot3(0, 0, 0, 'ro');

    %% process image
    % find camera extrinsics
    frame = undistortImage(frame, cameraParameters);
    imagePoints = detectCheckerboardPoints(frame);
    
    % if we didn't find a checkerboard, continue
    if (length(imagePoints) ~= nImagePoints)
        continue;
    end
    
    [rotation, translation] = extrinsics(imagePoints, worldPoints, cameraParameters);

    % find camera pose
    orientation = rotation';
    location = -translation/rotation;

%     % correct pose
%     V = rotationMatrixToVector(orientation);
%     R = makehgtform('zrotate', -2 * V(3));
%     R = R(1:3,1:3);
% 
%     location = location*X;
%     orientation = X*R*orientation;
%     location(1) = -location(1);

    % plot camera
    plotCamera('Location', location, 'Orientation', orientation, 'Opacity', 0.25, 'AxesVisible', true, 'Size', 5);

    %% viewing preferences
    grid on;
    axis equal
    axis vis3d
    axis([-120, 120, -120, 80, -300, 0]);
    view(-10, -80);
%     set(gca, 'CameraUpVector', [0 0 -1]);
    hold off;
    drawnow;
end
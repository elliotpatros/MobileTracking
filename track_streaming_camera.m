%% reset
clear all;
addpath(genpath('.'));

%% user parameters
ip_addr = '192.168.0.12';
squareSize = 20; % world units
worldUnits = 'mm';

%% set up
url = ['http://', ip_addr, ':8080/shot.jpg'];
imageFileNames = getImageFileNames('./Resources/checkerboard', 12, 'jpg');

%% calibrate camera
% detect checkerboards in images
[imagePoints, boardSize] = detectCheckerboardPoints(imageFileNames);

% get world coordinates of the corners
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% do calibration
cameraParameters = estimateCameraParameters(...
    imagePoints, worldPoints,...
    'EstimateSkew', true, ...
    'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, ...
    'WorldUnits', worldUnits, ...
    'InitialIntrinsicMatrix', [], ...
    'InitialRadialDistortion', []);
    

%% stream
try
    frame = imread(url);
    is_streaming = true;
catch
    error(['couldn"t connect to url: ', url]);
    is_streaming = false;
end

% frame_handle = image(frame);
while(is_streaming)
    % get current frame from video stream
    try
        frame = imread(url);
    catch
        is_streaming = false;
        disp('stream closed');
        break;
    end
    
    % undistort image
    undistortedFrame = undistortImage(frame, cameraParameters);
    
    % get camera position
    [rotationMatrix, translationVector] = extrinsics(imagePoints, worldPoints, cameraParameters)
    
    
%     set(frame_handle, 'CData', frame);
%     drawnow;
end

imageFileNames = getImageFileNames('~/Resources/checkerboard', 12, 'jpg');

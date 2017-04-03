%% reset
clear all;
addpath(genpath('.'));

%% user parameters
ip_addr = '192.168.0.12';
squareSize = 8; % world units (its actually 20)
worldUnits = 'mm';
needs_calibration = false;

%% calibrate camera
if (needs_calibration)
imageFileNames = getImageFileNames('./Resources/checkerboard', 12, 'jpg');
    
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
    'EstimateSkew', true, ...
    'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, ...
    'WorldUnits', worldUnits, ...
    'InitialIntrinsicMatrix', [], ...
    'InitialRadialDistortion', []);

end    

%% stream
disp('opening stream...');

% get first frame of stream
url = ['http://', ip_addr, ':8080/shot.jpg'];
try
    frame = imread(url);
    is_streaming = true;
catch
    error(['couldn"t connect to url: ', url]);
    is_streaming = false;
end

% process frames as fast as we can
while(is_streaming)
    % get current frame from video stream
    try
        frame = imread(url);
    catch
        is_streaming = false;
        disp('stream closed');
        break;
    end
    
    % find checkerboard points in this frame
    imagePoints = detectCheckerboardPoints(rgb2gray(frame));
    
    % if we didn't find a checkerboard, continue
    if (length(imagePoints) ~= nImagePoints)
        continue;
    end
    
    % undistort image
    undistortedFrame = undistortImage(frame, cameraParameters);
    
    % get camera position
    [rotationMatrix, translationVector] = extrinsics(imagePoints, worldPoints, cameraParameters);
    [orientation, location] = extrinsicsToCameraPose(rotationMatrix, translationVector);

    % plot...
    % ...world points
    plot3(worldPoints(:,1), worldPoints(:,2), zeros(size(worldPoints, 1), 1), '*');
    hold on;
    plot3(0,0,0,'ro');

    % ...camera
    plotCamera('Location', location, 'Orientation', orientation, 'Opacity', 0, 'Size', 20);

    % ...viewing preferences
    grid on
    axis equal
    axis manual
    set(gca, 'CameraUpVector', [0 0 -1]);
    camorbit(gca, 30, 0, 'data', [0 0 1]);
    cameratoolbar('SetMode','orbit');

    hold off;
    
    drawnow;
end

imageFileNames = getImageFileNames('~/Resources/checkerboard', 12, 'jpg');

%% user parameters
ip_addr = '192.168.0.12';

%% set up stream
url = ['http://', ip_addr, ':8080/shot.jpg'];
is_streaming = false;

%% stream
try
    frame = imread(url);
    is_streaming = true;
catch
    error(['couldn"t connect to url: ', url]);
end

frame_handle = image(frame);
while(is_streaming)
    try
        frame = imread(url);
    catch
        is_streaming = false;
        disp('stream closed');
        break;
    end
    
    set(frame_handle, 'CData', frame);
    drawnow;
end
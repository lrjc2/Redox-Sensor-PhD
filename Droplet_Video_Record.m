% Remove all previous video setups 
clc; clear;
imaqreset
close all; objects = imaqfind;%find video input objects in memory
delete(objects) %delete a video input object from memory
%% Specify File Prefix
FilePrefix = 'FILENAME';

%% Specify Number of seconds for each recording 
SecondsPerRecording = 60;
%% Specify number of seconds delay time and number of recordings per section 
SectionDelayTime = zeros(3,1); %Array to hold delay times of each section
NumRecordings = zeros(3,1); %Array to hold number of recordings at each section

SectionDelayTime(1) = 15*60-SecondsPerRecording; %Time between recordings in seconds %15  minutes
SectionDelayTime(2) = 1*60*60-SecondsPerRecording; %Time between recordings in seconds %1 hour
SectionDelayTime(3) = 3*60*60-SecondsPerRecording; %Time between recordings in seconds %3 hour

NumRecordings(1) = 9; %recordings in 1st section
NumRecordings(2) = 22; %recordings in 2nd section
NumRecordings(3) = 8*14; %recordings in 3rd section (8 per day, 14 days)


%% Crete array for function to use for delay times 
numVideos = sum(NumRecordings); %Calculate total number of videos 

%Create array with the delay time after each video 
DelayTime = zeros(numVideos,1);

%Fill with delay times from section array 
SectionStart = 1;
for n=1:length(SectionDelayTime)
    SectionEnd = SectionStart+NumRecordings(n)-1;
    DelayTime(SectionStart:SectionEnd) = SectionDelayTime(n);
    SectionStart = SectionEnd+1;
end
%% Specify Camera Source
vid = videoinput('winvideo', 1, 'MJPG_1920x1080');
src = getselectedsource(vid);

%% Specify ROI 
% vid.ROIPosition = [25 9 1207 690];

%% Setup recording frame rate 
FrameRate = 30; %FPS (Framve rate of camera)
FrameGrabInterval = 10; %Records every 10th frame 
%Set frames per trigger to correct ammount
vid.FramesPerTrigger = SecondsPerRecording*FrameRate/FrameGrabInterval;
vid.FrameGrabInterval = FrameGrabInterval; 

%Cell array to hold videos timestamped file name and metadata (Frame information)
metadataAll = cell(2, numVideos);

%% Start recording 
[metadataAll] = schedulerecording(vid, FilePrefix, SecondsPerRecording, DelayTime, FrameRate, FrameGrabInterval, metadataAll);

%% Close and Stop Video
stoppreview(vid);
closepreview(vid);
stop(vid);
delete(vid)


%% Write function to schedule recordings 
%Video and timestamps saved to file within function 
%Return the time aquisition started so filenames can be logged
function [metadataAll] = schedulerecording(vid, FilePrefix, SecondsPerRecording, DelayTime, FrameRate, FrameGrabInterval, metadataAll); 
  %Open Video for Viewing 
   preview(vid);
   pause(5);
% Setup Video Logging - Videos stores seqentially in the same file 
for n=1:length(DelayTime)
    vid.LoggingMode = 'disk&memory';
    timenow = datestr(now,'yyyy-mm-dd_hh-MM-ss');
    filename = [FilePrefix,'_', timenow]; %Name video file
    diskLogger = VideoWriter([filename,'.avi']);
    diskLogger.FrameRate = FrameRate/FrameGrabInterval;
    diskLogger.Quality = 100;
    vid.DiskLogger = diskLogger;
    
    %Trigers to be supplied manually with no limit 
    triggerconfig(vid, 'manual');
    vid.TriggerRepeat = 0; %Take individual 60second videos and save  
   
    %Start video for aquisition, await triggures
    start(vid);
    trigger(vid); %Triggure a recording
    wait(vid, SecondsPerRecording*1.5, 'logging'); %Wait to finish 
    
    %Find and save Metadata
    numAvail = vid.FramesAvailable; %Check number of frames available 
    [frames, timeStamp, metadata] = getdata(vid, numAvail); %Store frame data (timestamps)
    metadata = struct2table(metadata);
    metadata.AbsTime = datetime(metadata.AbsTime, 'Format', 'yyyy-MM-dd HH:mm:ss.SSS');
    metadataAll{1,n} = filename; %Store vidoes filename  in cell array
    metadataAll{2,n} = metadata; %Store vidoes meta data in cell array
    % Save metadata variable 
    save([FilePrefix,'.mat'], 'metadataAll');
    %Save Metadata table
    writetable(metadata,[FilePrefix,'_', timenow,'.csv']);
    %%Stop Video and end aquisition
    stop(vid);
    %% Delay  
    pause(DelayTime(n));
  
  
end

end





Videofiles = dir('*.avi');
Framefiles = dir('*.csv');


numVideos = length(Videofiles); %%Get width of cell array  

%Create cell arrays to hold videos and frame data 
Videos = cell(1, numVideos);
Frames = cell(1, numVideos);

%Import videos and frame data into cell array, loop though stored file names 
for n=1:numVideos  
    Videos{1,n} = VideoReader(Videofiles(n).name);
    Frames{1,n} = readtable(Framefiles(n).name);
end

% Create new combined new video
videoPlayer = vision.VideoPlayer;
outputVideo = VideoWriter('Analysis/FILENAME_COMBINED.avi');
outputVideo.FrameRate = Videos{1,1}.FrameRate;
open(outputVideo);

%Loop though vidoes in cell array and play each frame into the new video 
for n=1:numVideos  
   while hasFrame(Videos{1,n})
    imgt = readFrame(Videos{1,n});    % read each frame  
    %convert to grayscale 
    imgt_gray = rgb2gray(imgt);
    % record new video
    writeVideo(outputVideo, imgt_gray);
   end
end
close(outputVideo);

%Cycle though frame data tables to correct the frame number 
%Get last frame number of 1st video
LastFrameNum = max(Frames{1,1}.FrameNumber);
for n=2:numVideos  
    Frames{1,n}.FrameNumber = Frames{1,n}.FrameNumber + LastFrameNum;
    LastFrameNum = max(Frames{1,n}.FrameNumber);
end

%Verticaly concatinate corrected frame data tables into 1 table 
CombinedFrames = vertcat(Frames{:});
writetable(CombinedFrames,['Analysis/FILENAME_COMBINED','.csv']);






%%Version Log

%V01-01 Added section to remove oultiers/spikes from data using median filter.
    %And section to find stable force regions and only plot pulses from this region. 
    
    %V01-02- Added section to save figures and workspace. Cahged input to a folder and seperate filenames 
      %V01-02- Fixed datetiem importing options to fix find peaks issue in v02. Also changed the graph symbols to be different 

close all 
clear all
%% Import data
%FILENAME - Use a CSV file in the default Megunolink output(Load-Cell-Rig-Software-V01-00)
%Absolute Time, Flow Sensor 1 (ul/min), flow sensor 2 (ul/min), Force (N)
FOLDER = 'FOLDER';
FILENAME = 'FILENAME';
graphTitle = 'FILENAME';

PATH = [string(FOLDER) + '/' + string(FILENAME)];

opts = detectImportOptions(PATH+'-T1.csv');
opts.VariableNames = {'absTime', 'flow1', 'flow2', 'force'};
%Set abs time to datetime format
opts = setvartype(opts,{'absTime'},'datetime');
opts = setvaropts(opts,{'absTime'},'DatetimeFormat','dd-MMM-yyy HH:mm:ss.SSS', 'DatetimeLocale', 'en_GB' );
data=readtable(PATH+'-T1.csv', opts);


opts = detectImportOptions(PATH+'-T2.csv');
opts.VariableNames = {'absTime', 'flow1', 'flow2', 'force'};
%Set abs time to datetime format
opts = setvartype(opts,{'absTime'},'datetime');
opts = setvaropts(opts,{'absTime'},'DatetimeFormat','dd-MMM-yyy HH:mm:ss.SSS', 'DatetimeLocale', 'en_GB' );
dataT2=readtable(PATH+'-T2.csv', opts);

%Data reference 
%Line 1: data.flow1
%Line 2: dataT2.flow1
%Line 3: data.flow2
%Line 4: dataT2.flow2


%% Process Data
[line1Pulses, data] = processPulses(data,1,5,'flow1'); %(Data, figure to plot peak data, pump RPM, 'variable to process')
[line2Pulses, dataT2] = processPulses(dataT2,2,5,'flow1'); %(Data, figure to plot peak data, pump RPM, 'variable to process')
[line3Pulses, data] = processPulses(data,3,5,'flow2'); %(Data, figure to plot peak data, pump RPM, 'variable to process')
[line4Pulses, dataT2] = processPulses(dataT2,4,5,'flow2'); %(Data, figure to plot peak data, pump RPM, 'variable to process')
%% Define plot colours 
colors = brewermap(6,'Set1') ;
Markers = '*+o<>^dh.psvx';
%% Plot Raw Data T1
figure(5)

plot(data.absTime, data.flow1,  'Color', colors(1,:),'LineStyle' ,':')
hold on
plot(data.absTime, data.flow2,  'Color', colors(2,:),'LineStyle' ,':')
plot(data.absTime, data.flow1Clean,  'Color', colors(1,:),'Marker', '.', 'LineStyle' ,'-')
plot(data.absTime, data.flow2Clean,  'Color', colors(2,:),  'Marker', '.', 'LineStyle' ,'-')
plot(data.absTime, data.force,  'Color', colors(5,:))
% Create Title
title('Raw Data for: ' + string(FILENAME) + '-T1' )
% Create ylabel
ylabel('Flow Rate (μl/min) and Force (N)');
% Create xlabel
xlabel('Absolute Time');
% Create customizable objects that won't appear on the axes
% but will appear in the legend
h(1) = plot(nan, nan,'Marker', Markers(1), 'MarkerSize', 30,'LineWidth', 3, 'Color', colors(1,:), 'DisplayName', 'Line 1 (Flow Sensor 1) - Filtered');
h(2) = plot(nan, nan,'Marker', Markers(2),'MarkerSize', 30, 'LineWidth', 3, 'Color', colors(2,:),  'DisplayName', 'Line 3 (Flow Sensor 2) - Filtered');
h(3) = plot(nan, nan, 'LineWidth', 3, 'Color', colors(5,:),  'DisplayName', 'Load Cell');
legend(h)
legend(h, 'Location','northwest')
set(gca, 'FontSize', 24);
grid on
%%Set Y limits
ylim([-60,120]);
% Save figure
figFileName = [string(FOLDER) + "/" + string(FILENAME) + string('-RawT1')  + ".fig"];
savefig(figFileName);

%% Plot Raw Data T2
figure(6)
plot(dataT2.absTime, dataT2.flow1,  'Color', colors(3,:), 'LineStyle' ,':')
hold on
plot(dataT2.absTime, dataT2.flow2,  'Color', colors(4,:), 'LineStyle' ,':')
plot(dataT2.absTime, dataT2.flow1Clean,  'Color', colors(3,:), 'Marker', '.', 'LineStyle' ,'-')
plot(dataT2.absTime, dataT2.flow2Clean,  'Color', colors(4,:),   'Marker', '.', 'LineStyle' ,'-')

plot(dataT2.absTime, dataT2.force,  'Color', colors(5,:))
% Create Title
title('Raw Data for: ' + string(FILENAME) + '-T2' )
% Create ylabel
ylabel('Flow Rate (μl/min) and Force (N)');
% Create xlabel
xlabel('Absolute Time');
% Create customizable objects that won't appear on the axes
% but will appear in the legend
h(1) = plot(nan, nan,'Marker', '.','MarkerSize', 30,'LineWidth', 3, 'Color', colors(3,:), 'DisplayName', 'Line 2 (Flow Sensor 1) - Filtered');
h(2) = plot(nan, nan,'Marker', '.','MarkerSize', 30, 'LineWidth', 3, 'Color', colors(4,:),  'DisplayName', 'Line 4 (Flow Sensor 2) - Filtered');
h(3) = plot(nan, nan, 'LineWidth', 3, 'Color', colors(5,:),  'DisplayName', 'Load Cell');
legend(h, 'Location','northwest')
set(gca, 'FontSize', 24);
grid on
%Set Y limits
ylim([-60,120]);
% Save figure
figFileName = [string(FOLDER) + "/" + string(FILENAME) + string('-RawT2')  + ".fig"];
savefig(figFileName);


%% Create clean versions of pulse data (Only for regions of stable force) 
line1PulsesClean = line1Pulses(line1Pulses.forceStd <mean(data.forceStd),:);
line2PulsesClean = line2Pulses(line2Pulses.forceStd <mean(dataT2.forceStd),:);
line3PulsesClean = line3Pulses(line3Pulses.forceStd <mean(data.forceStd),:);
line4PulsesClean = line4Pulses(line4Pulses.forceStd <mean(dataT2.forceStd),:);


%% Tiled Plots of results (Flow)
figure
t = tiledlayout(1,2);
%title(t,string(graphTitle), 'FontSize', 22)
markerSize = 50;
% Plot pulse volume Tile 1
nexttile
scatter(line1PulsesClean.aveForce, line1PulsesClean.pulseVolume, markerSize, 'Marker', Markers(1), 'MarkerEdgeColor', colors(1,:))
hold on
scatter(line2PulsesClean.aveForce, line2PulsesClean.pulseVolume, markerSize, 'Marker', Markers(2), 'MarkerEdgeColor', colors(3,:))
scatter(line3PulsesClean.aveForce, line3PulsesClean.pulseVolume, markerSize, 'Marker', Markers(3), 'MarkerEdgeColor', colors(2,:))
scatter(line4PulsesClean.aveForce, line4PulsesClean.pulseVolume, markerSize, 'Marker', Markers(4), 'MarkerEdgeColor', colors(4,:))
xlimits = xlim; 

%Add reference line at y=0
hline = refline([0 0]);
hline.XData = [-1000,1000]; 
hline.Color  = 'k';
%Reset xlimits
xlim([0,120]);
% Create Title
title('A - Pulse Volumes'  )
% Create ylabel
ylabel('Flow Pulse Volume (μl)');
% Create xlabel
xlabel('Pump Bed Average Force (N)');
% Create customizable objects that won't appear on the axes
% but will appear in the legend
p(1) = plot(nan, nan, 'Marker', Markers(1), 'MarkerSize',500, 'LineWidth', 2, 'LineStyle','none', 'Color', colors(1,:), 'DisplayName', 'Line 1');
p(2) = plot(nan, nan, 'Marker', Markers(2), 'MarkerSize',500, 'LineWidth', 2, 'LineStyle','none', 'Color', colors(3,:),  'DisplayName', 'Line 2');
p(3) = plot(nan, nan, 'Marker', Markers(3), 'MarkerSize',500, 'LineWidth', 2, 'LineStyle','none', 'Color', colors(2,:), 'DisplayName', 'Line 3');
p(4) = plot(nan, nan, 'Marker', Markers(4), 'MarkerSize',500, 'LineWidth', 2, 'LineStyle','none', 'Color', colors(4,:),  'DisplayName', 'Line 4');
legend(p, 'location', 'northwest')
set(gca, 'FontSize', 20);
grid on

% Plot Peak Flow volume 
nexttile
scatter(line1PulsesClean.aveForce, line1PulsesClean.peakFlow, markerSize, 'Marker', Markers(1), 'MarkerEdgeColor', colors(1,:))
hold on
scatter(line2PulsesClean.aveForce, line2PulsesClean.peakFlow, markerSize, 'Marker', Markers(2), 'MarkerEdgeColor', colors(3,:))
scatter(line3PulsesClean.aveForce, line3PulsesClean.peakFlow, markerSize, 'Marker', Markers(3), 'MarkerEdgeColor', colors(2,:))
scatter(line4PulsesClean.aveForce, line4PulsesClean.peakFlow, markerSize, 'Marker', Markers(4), 'MarkerEdgeColor', colors(4,:))

%Negative peak
scatter(line1Pulses.aveForce, line1Pulses.peakNegFlow, markerSize, 'Marker', Markers(1), 'MarkerEdgeColor', colors(1,:))
scatter(line2PulsesClean.aveForce, line2PulsesClean.peakNegFlow, markerSize, 'Marker', Markers(2), 'MarkerEdgeColor', colors(3,:))
scatter(line3PulsesClean.aveForce, line3PulsesClean.peakNegFlow, markerSize, 'Marker', Markers(3), 'MarkerEdgeColor', colors(2,:))
scatter(line4PulsesClean.aveForce, line4PulsesClean.peakNegFlow, markerSize, 'Marker', Markers(4)', 'MarkerEdgeColor', colors(4,:))
%Add reference line at y=0
hline = refline([0 0]);
hline.XData = [-1000,1000]; 
hline.Color  = 'k';
%Reset xlimits
xlim([0,120]);
% Create Title
title('B - Peak Flow Rates ' )
% Create ylabel
ylabel('Peak Flow Rate (μl/min)');
% Create xlabel
xlabel('Pump Bed Average Force (N)');
set(gca, 'FontSize',20);
grid on

% Save figure
figFileName = [string(FOLDER) + "/" + string(FILENAME) + string('-Processed')  + ".fig"];
savefig(figFileName);



%% Force Variability Plot
figure
markerSize = 50;
scatter(line1PulsesClean.aveForce, line1PulsesClean.forceRange, markerSize, 'o', 'MarkerEdgeColor', colors(1,:))
hold on
scatter(line2PulsesClean.aveForce, line2PulsesClean.forceRange, markerSize, 'o', 'MarkerEdgeColor', colors(3,:))
scatter(line3PulsesClean.aveForce, line3PulsesClean.forceRange, markerSize, 'o', 'MarkerEdgeColor', colors(2,:))
scatter(line4PulsesClean.aveForce, line4PulsesClean.forceRange, markerSize, 'o', 'MarkerEdgeColor', colors(4,:))
xlimits = xlim; 

%Add reference line at y=0
hline = refline([0 0]);
hline.XData = [-1000,1000]; 
hline.Color  = 'k';
%Reset xlimits
xlim([0,120]);
% Create Title
title(['Force Range vs Average Force: ' + string(graphTitle)] )
% Create ylabel
ylabel('Force Range in Pulse (N)');
% Create xlabel
xlabel('Pump Bed Average Force (N)');
% Create customizable objects that won't appear on the axes
% but will appear in the legend
p(1) = plot(nan, nan, 'Marker', 'o', 'MarkerSize',500, 'LineWidth', 2, 'LineStyle','none', 'Color', colors(1,:), 'DisplayName', 'Line 1');
p(2) = plot(nan, nan, 'Marker', 'o', 'MarkerSize',500, 'LineWidth', 2, 'LineStyle','none', 'Color', colors(3,:),  'DisplayName', 'Line 2');
p(3) = plot(nan, nan, 'Marker', 'o', 'MarkerSize',500, 'LineWidth', 2, 'LineStyle','none', 'Color', colors(2,:), 'DisplayName', 'Line 3');
p(4) = plot(nan, nan, 'Marker', 'o', 'MarkerSize',500, 'LineWidth', 2, 'LineStyle','none', 'Color', colors(4,:),  'DisplayName', 'Line 4');
legend(p, 'location', 'northwest')
set(gca, 'FontSize', 18);
grid on
% Save figure
figFileName = [string(FOLDER) + "/" + string(FILENAME) + string('-ForceVariablity')  + ".fig"];
savefig(figFileName);


%% Save Workspace
workspaceFileName = [string(FOLDER) + '/' + string(FILENAME) + "-Analysis"];
save(workspaceFileName);

%% Create function for procesing data
function [pulseData, data] = processPulses(data, FigureNumber, pumpRPM, variable)
   % Remove spikes/outliers using median filter
   data.([string(variable)+'Clean']) = medfilt1(data.(variable),5);
   
   %Process force data to find the 5min force plateuas 
   %Calculate num sample to use as filter windows 
   sampleFreq = 40;%Hz
   sampleInterval = 1/sampleFreq; %s
   filterWindow = 60;%s
   filterWindowSamples = filterWindow/sampleInterval;
   %Apply moving mean filter to force data 
   data.forceFilter = movmean(data.force, filterWindowSamples);
   %Apply long std filter to find flat regions (areas with low std)
   data.forceStd = stdfilt(data.forceFilter, true(111));
   

%Matlab find peaks 

    %Calculate min peak distance from pump RPM
    minPeakDistance =  seconds(60/pumpRPM*0.8); %exected number of seconds between each peak with some tollerance 
    %Specify min peak prominance
    minPeakProm = 1; 
    
    %Plot Identified peaks for debugging 
    figure(FigureNumber)
    findpeaks(data.([string(variable)+'Clean']), data.absTime,'MinPeakProminence',minPeakProm, 'MinPeakDistance',minPeakDistance, 'Annotate', 'peaks'); 
    hold on
    title('Flow Sensor - Identified Peaks')
    ylabel('Flow Rate (μl/min)')
    xlabel('Absolute Time')
    grid on; 

   
    
    %Get peaks data 
    [pks,locs,widths,proms] = findpeaks(data.([string(variable)+'Clean']),data.absTime,'MinPeakProminence',minPeakProm, 'MinPeakDistance', minPeakDistance);
   
    %Find the number of pulses (Between peaks)
    numPulses = length(pks)-2; %Discard 1st and last peak 
    
    %Create a table to hold data about each pulse 
    pulseData = array2table(zeros(numPulses,5), 'VariableNames', {'pulseNum', 'peakFlow', 'pulseVolume', 'aveForce', 'peakNegFlow' });
   
    %Fill pulse number
    pulseData.pulseNum(:) = 1:1:numPulses;
    %Fill peak flow data - Note that pulse n contains peak (n+1)
    pulseData.peakFlow(:) = pks(2:length(pks)-1);
    %Fill peak time data - Note that pulse n contains peak (n+1)
    pulseData.peakTime(:) = locs(2:length(pks)-1);
    
    %Loop though pulses and find, Flow volume, Ave force, and peak negative flow rate
    for n=1:numPulses
        startTime = mean(locs(n:n+1)); %Start time defined as average between previous peak and peak within pulse 
        endTime = mean(locs(n+1:n+2)); %end time defined as average between peak within pulse and next peak 
        %get pulse length
        pulseData.pulseLength(n) = endTime-startTime;
        
        %Get flow and time data between times to get volume of pulse 
        tempFlow = data.([string(variable)+'Clean'])(data.absTime > startTime & data.absTime <endTime);
        tempTime= data.absTime(data.absTime > startTime & data.absTime <endTime);
        
        %Convert time to minutes for integration and integrate
        tempTime = datenum(tempTime-min(tempTime))*24*60; % Datenum returns days, convert to minutes
        pulseData.pulseVolume(n) = trapz(tempTime, tempFlow);%Integrate ul/min over minutes to give total flow volume in ul. 
        
        %Get Average Force 
        pulseData.aveForce(n) = mean(data.force(data.absTime > startTime & data.absTime <endTime));
       
        %Get Force Range 
        pulseData.forceRange(n) = range(data.force(data.absTime > startTime & data.absTime <endTime));
       
        %Get Force diff 
        pulseData.forceStd(n) = mean(data.forceStd(data.absTime > startTime & data.absTime <endTime));

        %Get peak negative flow rate 
        pulseData.peakNegFlow(n) = min(data.([string(variable)+'Clean'])(data.absTime > startTime & data.absTime <endTime));
    end
    
 
  end



    



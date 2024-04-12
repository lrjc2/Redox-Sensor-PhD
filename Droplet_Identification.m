%% Version Control

close all
clear variables

%% Preset graph fonts 
fontSize= 24;
fontName =  'Arial';

%FILENAME - Use a CSV file in the default Megunolink output
%Absolute Time, PD1, PD2, PD3, PD4, Temp1, Temp2 
FILENAME = 'FILENAME.csv';
output = 'OUTPUT';

%Droplet thresholds 
oil_drop_value_threshold_low = 850;
oil_drop_value_threshold_high = 990;
oil_norm = 900;
difference_threshold = 10;

% flow cell offset 
flow_cell_offset=0;%s


%Active Photodiodes - Set to 1 to analyse and 0 to not
PD1_Active = 0;
PD2_Active = 0;
PD3_Active = 1;
PD4_Active = 0;

%Pump settings
pump_speed = 20; %RPM
drop_per_rev = 1; %Numer of droplets per pump revolution 

%display processing graphs? Yes/no?
graphson=1;


%% END OF USER INPUTS





%% Import data

    data=readtable(FILENAME,'Delimiter',',');
    data.Properties.VariableNames = {'absTime', 'ADCResolution','ADCAverage', 'PD1', 'PD2', 'PD3', 'PD4', 'Brightness1' ,'Brightness2', 'Brightness3', 'Brightness4', 'Sensor1Average', 'Sensor2Average', 'Temperature 1', 'Temperature 2', 'motor1Speed', 'motor2Speed','ambientTemperature', 'ambientHumidity'};





%% Add a relaitve time coloumn 
data.relTime = datenum(data.absTime-min(data.absTime))*24*60*60;   %datenum return the time difference in days, scaling converts to seconds 


   

   %% scale values to 10 bit
    scale_resolution = 10;
    %Find scaling factor for each row
    scale_factor = (2^scale_resolution)./(2.^data.ADCResolution);
    %scale the PD and sensor rows 
    tempScaledCols = data{:,["PD1","PD2", "PD3", "PD4","Sensor1Average", "Sensor2Average"]}.*scale_factor;
     data(:,["PD1","PD2", "PD3", "PD4","Sensor1Average", "Sensor2Average"]) = array2table(tempScaledCols);
    %Update resolution col
     data.ADCResolution(:) = scale_resolution;





%% Raw data figure 
% Create figure
figure()

% Create axes
% Create multiple lines using matrix input to plo
plot(data.absTime,data.PD3,'b');


% Create ylabel
ylabel('ADC Value');
% Create xlabel
xlabel('Absolute Time (hh:mm)');


% Set the remaining axes properties
set(gca, 'FontName', fontName, 'FontSize', fontSize)
% Create legend
legend('UT6 Flow Cell');



 

%% Analyse data 

if PD1_Active == 1
    PlotNumber = 10;
    [PD1_Output, PD1_Thresholds] = FindDroplets(data.PD1, data.relTime, data.absTime, 'PD1', graphson, pump_speed, drop_per_rev, PlotNumber, oil_drop_value_threshold_low, oil_drop_value_threshold_high, oil_norm, difference_threshold);
    %% Save Output 
    writetable(PD1_Output, 'Output_PD1.csv');
end

if PD2_Active == 1
    PlotNumber = 20;
    [PD2_Output, PD2_Thresholds] = FindDroplets(data.PD2, data.relTime, data.absTime, 'PD2', graphson, pump_speed, drop_per_rev, PlotNumber, oil_drop_value_threshold_low, oil_drop_value_threshold_high, oil_norm, difference_threshold);
    %% Save Output 
    writetable(PD2_Output, 'Output_PD2.csv');
end

if PD3_Active == 1
    PlotNumber = 30;
    [PD3_Output, PD3_Thresholds] = FindDroplets(data.PD3, data.relTime-flow_cell_offset, data.absTime-seconds(flow_cell_offset), 'PD3', graphson, pump_speed, drop_per_rev, PlotNumber, oil_drop_value_threshold_low, oil_drop_value_threshold_high, oil_norm, difference_threshold);
        %% Save Output 
    writetable(PD3_Output, 'Output_PD3.csv');
end

if PD4_Active == 1
    PlotNumber = 40;
    [PD4_Output, PD4_Thresholds] = FindDroplets(data.PD4, data.relTime, data.absTime, 'PD4', graphson,pump_speed, drop_per_rev, PlotNumber, oil_drop_value_threshold_low, oil_drop_value_threshold_high, oil_norm, difference_threshold);
            %% Save Output 
    writetable(PD4_Output, 'Output_PD4.csv');
end



%% Create function for analysisng data 
function [Droplet_Output, Threshold_Output] = FindDroplets(working_data,relTime,absTime, name, graphson, pump_speed, drop_per_rev, PlotNumber, oil_drop_value_threshold_low, oil_drop_value_threshold_high, oil_norm, difference_threshold) %(Current PD data, Relative time stamp, absolute time stamp, Photodiode name,graphson, pump_speed, drop_per_rev )
    
%Find the difference between points to use as a threshold     
    a = length(working_data(:,1));
    Difference = zeros(a,1);
    for i = 2:a
        delta = abs(working_data(i)-working_data(i-1));
        Difference(i) = delta;
    end
   
% Plot to find threshold 
    figure(PlotNumber+1)
    plot(absTime,(working_data),'b',absTime,Difference ,'g')
    hold on    
    legend('Scaled Data', 'Difference')
    title(sprintf('Raw ADC data and Difference Threshold Plot - %s', name))   
    ylabel('Scaled ADC Value and Difference')
    xlabel('Time')
    grid on;   

Threshold_Output = table('Size',[1,3], 'VariableTypes',["double","double","double"],'VariableNames',["Oil_Water_Value_Threshold","Oil_Normalising_Value","Difference_Threshold"])  ;
Threshold_Output.Oil_Water_Value_Threshold = oil_drop_value_threshold_low;
Threshold_Output.Oil_Normalising_Value = oil_norm;
Threshold_Output.Difference_Threshold = difference_threshold;
%% Identify Oil droplets 
%Only keep data between than oil threshold value and with a difference less than difference_threshold (i.e. flat oil plateuas) 
keep_data = (Difference <difference_threshold) & (working_data > oil_drop_value_threshold_low) & (working_data < oil_drop_value_threshold_high); 
  
 %Create table to hold tempory oil droplet data, col1 - Absolute time, col2 - Relative time, col3 - value, col4 - drop number 
    OilDropletsTemp = table('Size',[sum(keep_data),4], 'VariableTypes',["datetime","double","double","double"],'VariableNames',["absTime","relTime","Value","DropletNumber"])  ;
    OilDropletsTemp.absTime = absTime(keep_data,:); %Store Absolute time in 1st coloum
    OilDropletsTemp.relTime = relTime(keep_data,:); %Store relative time in 2nd coloum
    OilDropletsTemp.Value = working_data(keep_data,:); %Store values in 3rd coloum
  
   %Number droplets and remove any isolated points 
  min_oil_drop_length = 5; %Minimum acceptable length of oil droplet in #samples
  max_oil_drop_sample_time_spacing = 0.2; %Any values not within 0.2s of another in the droplet will be deleted 
  drop_time_spacing = 1/(pump_speed/60*drop_per_rev); %Calculate the time between droplets in seconds - no user input needed  
  min_oil_drop_time_gap = drop_time_spacing * 0.75; %minimum acceptable droplet spaceing in seconds, calculated from expected drop spacing and halved to allow for wave shaping 
  drop_sample_count = 0;
  drop_number = 1;
  OilDropletsTemp.DropletNumber(1) = drop_number; %Store drop number in column 4
  current_drop_start_time =  OilDropletsTemp.relTime(1); %Variable to keep track of start time of current droplet, iniated by 1st value 
    
 %Number droplets
 for i = 2:height(OilDropletsTemp)
     if (OilDropletsTemp.relTime(i) - current_drop_start_time) >  min_oil_drop_time_gap %Assign new drop number if the time gap between points is large enough 
         drop_number=drop_number+1;
         current_drop_start_time = OilDropletsTemp.relTime(i); %Updated start time for new droplet
         OilDropletsTemp.DropletNumber(i) = drop_number;
     else
         OilDropletsTemp.DropletNumber(i) = drop_number;
     end
 end

    total_number_droplets = max(OilDropletsTemp.DropletNumber);
 
 %% Identify the flattest section of the oil droplets
    %Create table to hold finalised oil droplet data, col1 - Absolute time, col2 - Relative time, col3 - value, col4 - drop number 
    OilDroplets = table('Size',[total_number_droplets*16,4], 'VariableTypes',["datetime","double","double","double"],'VariableNames',["absTime","relTime","Value","DropletNumber"])  ;
    start_index=1;
    for n=1:max(OilDropletsTemp.DropletNumber)
        OilDropletsCurrent = OilDropletsTemp((OilDropletsTemp.DropletNumber==n),:);
        if height(OilDropletsCurrent)>16
            oil_droplet_sample_length = 16;
            standard_deviation = zeros(height(OilDropletsCurrent)-oil_droplet_sample_length,1);
            for j = 1:(height(OilDropletsCurrent)-oil_droplet_sample_length)
                standard_deviation(j) = std(OilDropletsCurrent.Value(j:j+oil_droplet_sample_length));
            end  
            [M, index] = min(abs(standard_deviation)); %Find the index with the minimun standard_deviation to give us the start of the 15 flattest points 
            OilDroplets(start_index:start_index+oil_droplet_sample_length,:) = OilDropletsCurrent(index:index+oil_droplet_sample_length,:);
            %Update start index
            start_index = start_index+oil_droplet_sample_length+1;
        else
            OilDroplets(start_index:start_index+height(OilDropletsCurrent)-1,:) =  OilDropletsCurrent(:,:); 
            %Update start index
            start_index = start_index+height(OilDropletsCurrent);
        end
    end
    

%% Indentify water droplets 
 %Create table to hold finalised water droplet data, col1 - Absolute time, col2 - Relative time, col3 - value, col4 - drop number 
WaterDroplets = table('Size',[total_number_droplets,4], 'VariableTypes',["datetime","double","double","double"],'VariableNames',["absTime","relTime","Value","DropletNumber"])  ;
    
% Take spacing between each oil droplet 
for i = 1:total_number_droplets
   
%Identify gaps between oil droplets 
    drop_start_time = max(OilDroplets.relTime(OilDroplets.DropletNumber==i)); %End time of current oil droplet
    if i <total_number_droplets
        drop_end_time =  min(OilDroplets.relTime(OilDroplets.DropletNumber==i+1)); %start time of next oil droplet
    elseif i == total_number_droplets
        drop_end_time = max(relTime);
    end
    %Select all the water droplet data for the currently identifed droplet. 
    keep_data_high = relTime>drop_start_time & relTime<drop_end_time & working_data>oil_drop_value_threshold_high & working_data<1000 & Difference <difference_threshold;
    keep_data_low = relTime>drop_start_time & relTime<drop_end_time & working_data<oil_drop_value_threshold_low & working_data<1000 & Difference <difference_threshold;
    keep_data = keep_data_high | keep_data_low;
    %%Tempory table for working water droplet data 
WaterDropletsTemp = table('Size',[sum(keep_data),4], 'VariableTypes',["datetime","double","double","double"],'VariableNames',["absTime","relTime","Value","DropletNumber"])  ;
       
    %Store time, data and drop number in the temporay variable 
    WaterDropletsTemp.absTime = absTime(keep_data,:); %Store absolute time in 1st coloum
    WaterDropletsTemp.relTime = relTime(keep_data,:); %Store relative time in 2nd coloum
    WaterDropletsTemp.Value = working_data(keep_data,:); %Store values in 3rd coloum
    WaterDropletsTemp.DropletNumber(:) = i; %Store drop number in 4th coloum    
    
    %Select middle 80% of points 
    if height(WaterDropletsTemp)>10
        remove_samples = round(height(WaterDropletsTemp)*((1-80/100)/2));
        WaterDropletsTemp(1:remove_samples,:) = [];
        WaterDropletsTemp(height(WaterDropletsTemp)-remove_samples:height(WaterDropletsTemp),:) = [];
    end  
    WaterDroplets = [WaterDroplets; WaterDropletsTemp(:,:)];
end

% % Plot oil and water points 
if graphson==1
figure(PlotNumber+2)
        plot(absTime,working_data,'b',OilDroplets.absTime,OilDroplets.Value,'r.',WaterDroplets.absTime,WaterDroplets.Value,'g.' )
        hold on
        title(sprintf('Raw ADC data and Identified Points - %s', name))
        ylabel('ADV Value')
        xlabel('Time (s)')
        grid on;
end

%% Measure intensity of each oil/water droplet 

%Create tables to hold oil and water average data 
oil_droplet_intensity = table('Size',[total_number_droplets,4], 'VariableTypes',["double","datetime","double","double"],'VariableNames',["DropletNumber","absTime","relTime","Value"])  ;
water_droplet_intensity = table('Size',[total_number_droplets,6], 'VariableTypes',["double","datetime","double","double","double","double"],'VariableNames',["DropletNumber","absTime","relTime","Value","COV", "Range"])  ;
for i = 2:total_number_droplets
    oil_droplet_intensity.DropletNumber(i) = i; %Droplet number
    oil_droplet_intensity.absTime(i) = mean(OilDroplets.absTime(OilDroplets.DropletNumber==i)); %Mean oil droplet Absolute time
    oil_droplet_intensity.relTime(i) = mean(OilDroplets.relTime(OilDroplets.DropletNumber==i)); %Mean oil droplet Relative time
    oil_droplet_intensity.Value(i) = mean(OilDroplets.Value(OilDroplets.DropletNumber==i)); %Mean oil droplet intensity
    water_droplet_intensity.DropletNumber(i) = i; %Droplet number
    water_droplet_intensity.absTime(i) = mean(WaterDroplets.absTime(WaterDroplets.DropletNumber==i)); %Mean water droplet Absolute time
    water_droplet_intensity.relTime(i) = mean(WaterDroplets.relTime(WaterDroplets.DropletNumber==i)); %Mean water droplet Relative time
    water_droplet_intensity.Value(i) = mean(WaterDroplets.Value(WaterDroplets.DropletNumber==i)); %Mean water droplet intensity
    water_droplet_intensity.COV(i) = std(WaterDroplets.Value(WaterDroplets.DropletNumber==i)); %std/mean water droplet intensity
    water_droplet_intensity.Range(i) = 100*replaceWith0IfEmpty(range(WaterDroplets.Value(WaterDroplets.DropletNumber==i)))/water_droplet_intensity.Value(i);%range water droplet intensity (normalised against intensity mean)

end  
   
%% Normalise Droplets to a set oil level 
 %Create tables to hold normalised values
    oil_droplet_intensity_norm = oil_droplet_intensity;
    water_droplet_intensity_norm = water_droplet_intensity;
        
    %Normalise oil droplet and water droplet by the same ammount
    for i = 1:height(oil_droplet_intensity)
            temp_scale_factor = oil_norm/oil_droplet_intensity.Value(i); %calculate scaling facotr
            oil_droplet_intensity_norm.Value(i) = oil_droplet_intensity.Value(i)*temp_scale_factor; %scale oil value            
            water_droplet_intensity_norm.Value(i) = water_droplet_intensity.Value(i)*temp_scale_factor; %Normalised water droplet value
            % Also Calcualte an absorbance relative to the oil normalised level
            water_droplet_intensity_norm.absorbance_relative_to_oil(i) = -log10(water_droplet_intensity_norm.Value(i)/oil_norm);
    end


% remove any nan 
water_droplet_intensity_norm(isnan(water_droplet_intensity_norm.Value),:) = [];

%% 

figure(PlotNumber+3)
subplot(2,1,1)
plot(absTime,working_data,'b',oil_droplet_intensity.absTime,oil_droplet_intensity.Value,'ro', water_droplet_intensity.absTime,water_droplet_intensity.Value,'go')
title(sprintf('Raw ADC data and Identified Droplets - %s', name))      
ylabel('ADC Value')
xlabel('Time (s)')
grid on;
legend(sprintf('Raw ADC Data - %s', name),'Oil Droplets Mean', 'Water Droplets Mean')  

subplot(2,1,2)
plot(absTime,working_data,'b',oil_droplet_intensity_norm.absTime,oil_droplet_intensity_norm.Value,'ro', water_droplet_intensity_norm.absTime,water_droplet_intensity_norm.Value,'go')
title(sprintf('Raw ADC data and Identified Normalised Droplets - %s', name))      
ylabel('ADC Value')
xlabel('Time (s)')
grid on;
legend(sprintf('Raw ADC Data - %s', name),'Oil Droplets Mean', 'Water Droplets Mean')  
Droplet_Output = water_droplet_intensity_norm;
end 

function x = replaceWith0IfEmpty(x)
if isempty(x)
    x = 0;
end
end














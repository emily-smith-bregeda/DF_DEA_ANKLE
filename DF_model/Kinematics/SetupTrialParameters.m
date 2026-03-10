%% A. Set up results folder and assign trial parameters (SetupTrialParameters.m)

function [folderPath, folderName, trialNumStr, prefix, side, resultsFolder, jointSelection, cutoff_freq, sampling_freq, heelStrikeFrame, toeOffFrame, numFrames, gaitCycle] = SetupTrialParameters()
% SetupTrialParameters - Prompts the user for trial folder, joints, frequencies, and gait frames

    % Prompt user to select a directory
    folderPath = uigetdir('','Select the trial folder (e.g., 001_Walking_R)');
    
    % Check if the user canceled the folder selection
    if folderPath == 0
        error('Folder selection canceled. Please select a valid directory.');
    end
    
    % Extract the trial number, prefix, and side from the folder name
    [~, folderName] = fileparts(folderPath);
    splitName = split(folderName, '_');
    trialNumStr = splitName{1};  % Trial number (e.g., '001')
    prefix = splitName{2};       % Trial type (e.g., 'Walking')
    side = splitName{3};         % Side (e.g., 'R' or 'L')
   
   % ----- %
   % comment this section out if you do NOT want an automatic results folder created 
    dateStr = datestr(now, 'yyyy-mm-dd');

    baseFolder = fullfile(folderPath, ['Results_' dateStr]);
    resultsFolder = baseFolder;
    suffix = 1;
    % If the results folder already exists for today, add a "_1"
    while exist(resultsFolder, 'dir')
        resultsFolder = [baseFolder '_' num2str(suffix)];
        suffix = suffix + 1;
    end

    mkdir(resultsFolder);
    disp(['Results folder created at: ' resultsFolder]);

    resultsFolder = folderPath;
    % ----- %
    % Select which joints to analyze
    jointSelection = menu('Select joints to analyze:', ...
        'Tibiotalar only', ...
        'Tibiotalar and Subtalar', ...
        'Tibiotalar, Subtalar, Talonavicular', ...
        'Tibiotalar, Subtalar, Talonavicular, and Calcaneocuboid', ...
        'Tibiotalar, Subtalar, Talonavicular, Calcaneocuboid and Tibiofibular');
    
    % Prompt user for cutoff frequency
    prompt = {'Enter the desired cutoff frequency (in Hz):', ...
              'Enter the sampling frequency (in Hz):'};
    dlgtitle = 'Frequency Input';
    dims = [1 35];  % Dimensions of the dialog box
    definput = {'12', '120'};  % Default values
    
    answer = inputdlg(prompt, dlgtitle, dims, definput);
    
    % Convert inputs from string to numeric
    cutoff_freq = str2double(answer{1});
    sampling_freq = str2double(answer{2});
    
    if isempty(cutoff_freq) || cutoff_freq <= 0
        error('Invalid input. Please enter a positive number for cutoff frequency.');
    end
    
    if isempty(sampling_freq) || sampling_freq <= 0
        error('Invalid input. Please enter a positive number for sampling frequency.');
    end
    
    % Ask user to input the frame number for Heel Strike and Toe Off
    heelStrike = inputdlg('Enter the frame number of Heel Strike:', 'Heel Strike Frame');
    toeOff = inputdlg('Enter the frame number of Toe Off:', 'Toe Off Frame');
    
    % Convert user input from string to numeric
    heelStrikeFrame = str2double(heelStrike{1});
    toeOffFrame = str2double(toeOff{1});
    
    if isnan(heelStrikeFrame) || isnan(toeOffFrame) || heelStrikeFrame <= 0 || toeOffFrame <= 0
        error('Invalid frame numbers entered. Please enter positive numeric values.');
    end
    
    % Save the Heel Strike and Toe Off frames in a structure
    gaitCycle.(prefix).frames.heelStrike = heelStrikeFrame;
    gaitCycle.(prefix).frames.toeOff = toeOffFrame;
    
    % Number of frames between heel strike and toe off
    numFrames = toeOffFrame - heelStrikeFrame + 1;
    
    if numFrames <= 0
        error('Toe Off frame must be greater than Heel Strike frame.');
    end

    disp(['Trial parameters have been successfully loaded for trial: ' folderName]);
end

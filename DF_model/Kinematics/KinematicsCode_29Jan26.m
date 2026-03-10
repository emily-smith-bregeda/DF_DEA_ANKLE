%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Foot and Ankle Kinematic Code
% This code takes the coordinate information, bone models, and autoscoper
% tracking data. It will output the translation and rotation for each bone,
% the joint angles, and animated GIFs of the motion
%
% 
% Written by Robert Chauvet/Emily Smith
% Last update: July 2025
% - Added Capability with interpolating between data Points that were
% missing
% - Checked
%
% Oragnized by Janae Ruzicki,  Sept 2025
% see the SOP for guidance
% updated joint angle calculations
%
% Based on
% Specific code for manual vertebral segmentation and axis relocation     
% Cervical Vertebrae C4-C5                                               
% By Tomasz Bugajski / Emily Bangsboll                                                                    
%
%                                            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Common Errors
% Load Bone_models and check what they are called:
% changes faces to vertices
% change Points to ConnectivityList



%%
close all;
clear;
clc;
%% A. Set up results folder and assign trial parameters (SetupTrialParameters.m)
[folderPath, folderName, trialNumStr, prefix, side, resultsFolder, jointSelection, cutoff_freq, sampling_freq, heelStrikeFrame, toeOffFrame, numFrames, gaitCycle] = SetupTrialParameters();

%% B. Set up local coordinate systems and  (BuildBoneModels.m)
[Bone_model, Bone_model_Auto, Coordinate, LCS] = BuildBoneModels(folderPath, jointSelection);

%% C. Visualize 3D bone models
% figure;
% pcshow(Bone_model.Tibia.Points(:,1:3), 'b'); hold on;
% pcshow(Bone_model.Talus.Points(:,1:3), 'r'); hold on;
% 
% if jointSelection > 1
%     pcshow(Bone_model.Calcaneus.Points(:,1:3), 'g'); hold on;
% end
% if jointSelection == 3
%     pcshow(Bone_model.Navicular.Points(:,1:3), 'c'); hold on;
% elseif jointSelection == 4
%     pcshow(Bone_model.Cuboid.Points(:,1:3), 'm'); hold on;
% end
% 
% % Display original coordinate systems
% vectarrow(LCS_Auto.TibiaTT.Origin, LCS_Auto.TibiaTT.X, 'r');hold on;
% vectarrow(LCS_Auto.TibiaTT.Origin,LCS_Auto.TibiaTT.Y, 'g');hold on;
% vectarrow(LCS_Auto.TibiaTT.Origin,LCS_Auto.TibiaTT.Z, 'b');hold on;
% 
% vectarrow(LCS_Auto.TalusTT.Origin, LCS_Auto.TalusTT.X, 'r');hold on;
% vectarrow(LCS_Auto.TalusTT.Origin,LCS_Auto.TalusTT.Y, 'g');hold on;
% vectarrow(LCS_Auto.TalusTT.Origin,LCS_Auto.TalusTT.Z, 'b');hold on;
% 
% xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
% title('3D Bone Models')
% axis equal;

%% D. Transform bones to local coordinate system (LCS) (Transform2LCS.m)
[Bone_model_LCS, Auto2LCS] = Transform2LCS(Bone_model, LCS,  jointSelection);

%% E. Visualize 3D bone models with LCS applied and aligned to the origin
% figure;
% pcshow(Bone_model_LCS.TibiaTT.Points(:,1:3), 'r'); hold on;
% pcshow(Bone_model_LCS.TalusTT.Points(:,1:3), 'b'); hold on;
% 
% if jointSelection > 1
%     pcshow(Bone_model_LCS.CalcaneusST.Points(:,1:3), 'g'); hold on;
%     pcshow(Bone_model_LCS.TalusST.Points(:,1:3), 'y'); hold on;
% end
% if jointSelection == 3
%     pcshow(Bone_model_LCS.NavicularTN.Points(:,1:3), 'c'); hold on;
% elseif jointSelection == 4
%     pcshow(Bone_model_LCS.CuboidCC.Points(:,1:3), 'm'); hold on;
% end
% 
% % Display LCS coordinate systems
% vectarrow(LCS_LCS.TibiaTT.Origin(1:3), LCS_LCS.TibiaTT.X(1:3), 'r');hold on;
% vectarrow(LCS_LCS.TibiaTT.Origin(1:3),LCS_LCS.TibiaTT.Y(1:3), 'g');hold on;
% vectarrow(LCS_LCS.TibiaTT.Origin(1:3),LCS_LCS.TibiaTT.Z(1:3), 'b');hold on;
% 
% vectarrow(LCS_LCS.TalusTT.Origin(1:3), LCS_LCS.TalusTT.X(1:3), 'r');hold on;
% vectarrow(LCS_LCS.TalusTT.Origin(1:3),LCS_LCS.TalusTT.Y(1:3), 'g');hold on;
% vectarrow(LCS_LCS.TalusTT.Origin(1:3),LCS_LCS.TalusTT.Z(1:3), 'b');hold on;
% 
% xlabel('X(m)'); ylabel('Y(m)'); zlabel('Z(m)');
% title('Transformed 3D bone models')
% axis equal;

%% F. Loading Dynamic and Neutral Tracking Data (LoadTRA.m)
[Tracking_Auto, Neutral_Auto, Tracking_Data, trackingFolder, Lengths, Starts] = LoadTRA(trialNumStr, side, folderName, folderPath, prefix, jointSelection);

%% Plot Neutral Trial
% ADD HERE - plotting function ***

%% G. Transform dynamic and neutral tracking to LCS (TransformTracking.m)
% transforms the neutral and the dynamic seperately
[Neutral_matx_LCS, Tracking_LCS, Angle_LCS, Translation_LCS] = TransformTracking(Neutral_Auto, Tracking_Auto, Auto2LCS, LCS, Lengths, prefix, jointSelection, side);

%% H. Filter LCS Tracking Matrices for Visualization (FilterTracking.m)
[Filtered_Tracking_LCS] = filterTransform4x4(Tracking_LCS, prefix, jointSelection,sampling_freq,6);

%% I. Produce Results --> Joint angles

nFrames = toeOffFrame - heelStrikeFrame + 1;
Norm_RelMat.(prefix).Tibiotalar = zeros(3,3,nFrames);
Norm_RelAngle.(prefix).Tibiotalar = zeros(nFrames,3);

Norm_RelMat.(prefix).Subtalar = zeros(3,3,nFrames);
Norm_RelAngle.(prefix).Subtalar = zeros(nFrames,3);

Norm_RelMat.(prefix).Talonavicular = zeros(3,3,nFrames);
Norm_RelAngle.(prefix).Talonavicular = zeros(nFrames,3);

Norm_RelMat.(prefix).Calcaneocuboid = zeros(3,3,nFrames);
Norm_RelAngle.(prefix).Calcaneocuboid = zeros(nFrames,3);

% neutral and dynamic joint angles SEPARATELY
% calculate joint angles for neutral and dynamic THEN neutralize

% neutral joint angles
% Tibiotalar
Neutral_RelMat.(prefix).Tibiotalar = Neutral_matx_LCS.(prefix).TibiaTT \ Neutral_matx_LCS.(prefix).TalusTT;
Neutral_RelAngle.(prefix).Tibiotalar = rad2deg(rotm2eul(Neutral_RelMat.(prefix).Tibiotalar(1:3,1:3), 'ZYX'));

% dynamic joint angles
for i = heelStrikeFrame:toeOffFrame
    % Tibiotalar
    Dynamic_RelMat.(prefix).Tibiotalar(:,:,i) = Filtered_Tracking_LCS.(prefix).TibiaTT(:,:,i) \ (Filtered_Tracking_LCS.(prefix).TalusTT(:,:,i));
    Dynamic_RelAngle.(prefix).Tibiotalar(i,:) = rad2deg(rotm2eul(Dynamic_RelMat.(prefix).Tibiotalar(1:3,1:3,i), 'ZYX'));
% Neutralize
    Norm_RelMat.(prefix).Tibiotalar(:,:,i) = Neutral_RelMat.(prefix).Tibiotalar(1:3,1:3) \ (Dynamic_RelMat.(prefix).Tibiotalar(1:3,1:3,i));
    Norm_RelAngle.(prefix).Tibiotalar(i,:) = rad2deg(rotm2eul(Norm_RelMat.(prefix).Tibiotalar(1:3,1:3,i), 'ZYX'));
end
disp('Tibiotalar joint angles calculated');

if jointSelection > 1
        % Subtalar ** XZY for main plane of motion as inversion/eversion (Y)
    Neutral_RelMat.(prefix).Subtalar = Neutral_matx_LCS.(prefix).TalusST \ Neutral_matx_LCS.(prefix).CalcaneusST;
    Neutral_RelAngle.(prefix).Subtalar = rad2deg(rotm2eul(Neutral_RelMat.(prefix).Subtalar(1:3,1:3), 'XZY'));
    for i = heelStrikeFrame:toeOffFrame
        Dynamic_RelMat.(prefix).Subtalar(:,:,i) = Filtered_Tracking_LCS.(prefix).TalusST(:,:,i) \ (Filtered_Tracking_LCS.(prefix).CalcaneusST(:,:,i));
        Dynamic_RelAngle.(prefix).Subtalar(i,:) = rad2deg(rotm2eul(Dynamic_RelMat.(prefix).Subtalar(1:3,1:3,i), 'XZY'));
    % Neutralize
        Norm_RelMat.(prefix).Subtalar(:,:,i) = Neutral_RelMat.(prefix).Subtalar(1:3,1:3) \ (Dynamic_RelMat.(prefix).Subtalar(1:3,1:3,i));
        Norm_RelAngle.(prefix).Subtalar(i,:) = rad2deg(rotm2eul(Norm_RelMat.(prefix).Subtalar(1:3,1:3,i), 'XZY'));
    
    end
    disp('Subtalar joint angles calculated');
end

if jointSelection >= 3
        % Talonavicular
    Neutral_RelMat.(prefix).Talonavicular = Neutral_matx_LCS.(prefix).TalusTN \ Neutral_matx_LCS.(prefix).NavicularTN;
    Neutral_RelAngle.(prefix).Talonavicular = rad2deg(rotm2eul(Neutral_RelMat.(prefix).Talonavicular(1:3,1:3), 'ZYX'));
    for i = heelStrikeFrame:toeOffFrame
        Dynamic_RelMat.(prefix).Talonavicular(:,:,i) = Filtered_Tracking_LCS.(prefix).TalusTN(:,:,i) \ (Filtered_Tracking_LCS.(prefix).NavicularTN(:,:,i));
        Dynamic_RelAngle.(prefix).Talonavicular(i,:) = rad2deg(rotm2eul(Dynamic_RelMat.(prefix).Talonavicular(1:3,1:3,i), 'ZYX'));
    % Neutralize
        Norm_RelMat.(prefix).Talonavicular(:,:,i) = Neutral_RelMat.(prefix).Talonavicular(1:3,1:3) \ (Dynamic_RelMat.(prefix).Talonavicular(1:3,1:3,i));
        Norm_RelAngle.(prefix).Talonavicular(i,:) = rad2deg(rotm2eul(Norm_RelMat.(prefix).Talonavicular(1:3,1:3,i), 'ZYX'));
    
    end
    disp('Talonavicular joint angles calculated');
end

if jointSelection >= 4
    % Calcaneocuboid
    Neutral_RelMat.(prefix).Calcaneocuboid = Neutral_matx_LCS.(prefix).CuboidCC \ Neutral_matx_LCS.(prefix).CalcaneusCC;
    Neutral_RelAngle.(prefix).Calcaneocuboid = rad2deg(rotm2eul(Neutral_RelMat.(prefix).Calcaneocuboid(1:3,1:3), 'ZYX'));
    for i = heelStrikeFrame:toeOffFrame
        Dynamic_RelMat.(prefix).Calcaneocuboid(:,:,i) = Filtered_Tracking_LCS.(prefix).CuboidCC(:,:,i) \ (Filtered_Tracking_LCS.(prefix).CalcaneusCC(:,:,i));
        Dynamic_RelAngle.(prefix).Calcaneocuboid(i,:) = rad2deg(rotm2eul(Dynamic_RelMat.(prefix).Calcaneocuboid(1:3,1:3,i), 'ZYX'));
    % Neutralize
        Norm_RelMat.(prefix).Calcaneocuboid(:,:,i) = Neutral_RelMat.(prefix).Calcaneocuboid(1:3,1:3) \ (Dynamic_RelMat.(prefix).Calcaneocuboid(1:3,1:3,i));
        Norm_RelAngle.(prefix).Calcaneocuboid(i,:) = rad2deg(rotm2eul(Norm_RelMat.(prefix).Calcaneocuboid(1:3,1:3,i), 'ZYX'));
    
    end
    disp('Calcaneocuboid joint angles calculated');
end

%% Interpolation
[Norm_RelAngle_interp] = InterpJointData(Norm_RelAngle, prefix, heelStrikeFrame, toeOffFrame, jointSelection);

%% TibioTalar Graphs
%Plot and Save the joint angle and translation plots

% Joint Angle
figure('Name','Tibiotalar Joint Angles');
sgtitle('Tibiotalar Joint Angles');

% Subplot 1
subplot(1, 3, 1);  % 1 row, 3 columns, 1st plot
plot(Norm_RelAngle_interp.(prefix).Tibiotalar(:,3), 'color', 'r', 'LineWidth', 2, 'LineStyle', '-');
xlabel('% Stance Phase');
ylabel('Joint Angle [{\circ}]');
title('Dorsiflexion (+) / Plantarflexion (-)');
set(gca, 'Color', 'w');
set(gca, 'box', 'off');

% Subplot 2
subplot(1, 3, 2);  % 1 row, 3 columns, 2nd plot
plot(Norm_RelAngle_interp.(prefix).Tibiotalar(:,2), 'color', 'r', 'LineWidth', 2, 'LineStyle', '-');
xlabel('% Stance Phase');
ylabel('Joint Angle [{\circ}]');
title('Eversion (+) / Inversion (-)');
set(gca, 'Color', 'w');
set(gca, 'box', 'off');

% Subplot 3
subplot(1, 3, 3);  % 1 row, 3 columns, 3rd plot
plot(Norm_RelAngle_interp.(prefix).Tibiotalar(:,1), 'color', 'r', 'LineWidth', 2, 'LineStyle', '-');
xlabel('% Stance Phase');
ylabel('Joint Angle [{\circ}]');
title('External Rotation (+) / Internal Rotation(-)');
set(gca, 'Color', 'w');
set(gca, 'box', 'off');
set(gcf, 'Position', [10 10 900 300]);

% Save Tibiotalar Joint Angles Figure

saveas(gcf, fullfile(resultsFolder, sprintf('Trial%s_Tibiotalar_Joint_Angles.png', trialNumStr)));

%% Subtalar Graphs
if jointSelection > 1
    % Plot and Save the joint angle and translation plots
    
    % Joint Angle
    figure('Name','Subtalar Joint Angles');
    sgtitle('Subtalar Joint Angles');
    
    % Subplot 1: Dorsiflexion/Plantarflexion
    subplot(1, 3, 1);  
    plot(Norm_RelAngle_interp.(prefix).Subtalar(:,2), 'color', 'g', 'LineWidth', 2, 'LineStyle', '-');
    xlabel('% Stance Phase');
    ylabel('Joint Angle [{\circ}]');
    title('Dorsiflexion (+) / Plantarflexion (-)');
    set(gca, 'Color', 'w');
    set(gca, 'box', 'off');
    
    % Subplot 2: Eversion/Inversion
    subplot(1, 3, 2);  
    plot(Norm_RelAngle_interp.(prefix).Subtalar(:,3), 'color', 'g', 'LineWidth', 2, 'LineStyle', '-');
    xlabel('% Stance Phase');
    ylabel('Joint Angle [{\circ}]');
    title('Eversion (+) / Inversion (-)');
    set(gca, 'Color', 'w');
    set(gca, 'box', 'off');
    
    % Subplot 3: External Rotation/Internal Rotation
    subplot(1, 3, 3);  
    plot(Norm_RelAngle_interp.(prefix).Subtalar(:,1), 'color', 'g', 'LineWidth', 2, 'LineStyle', '-');
    xlabel('% Stance Phase');
    ylabel('Joint Angle [{\circ}]');
    title('External Rotation (+) / Internal Rotation(-)');
    set(gca, 'Color', 'w');
    set(gca, 'box', 'off');
    set(gcf, 'Position', [10 10 900 300]);
    
    % Save Subtalar Joint Angles Figure
    saveas(gcf, fullfile(resultsFolder, sprintf('Trial%s_Subtalar_Joint_Angles.png', trialNumStr)));
    

end
%% Talonavicular Graphs
    % Plot and Save the joint angle and translation plots
if jointSelection >= 3
    % Joint Angle
    figure('Name','Talonavicular Joint Angles');
    sgtitle('Talonavicular Joint Angles');
    
    % Subplot 1: Dorsiflexion/Plantarflexion
    subplot(1, 3, 1);  
    plot(Norm_RelAngle_interp.(prefix).Talonavicular(:,3), 'color', 'm', 'LineWidth', 2, 'LineStyle', '-');
    xlabel('% Stance Phase');
    ylabel('Joint Angle [{\circ}]');
    title('Dorsiflexion (+) / Plantarflexion (-)');
    set(gca, 'Color', 'w');
    set(gca, 'box', 'off');
    
    % Subplot 2: Eversion/Inversion
    subplot(1, 3, 2);  
    plot(Norm_RelAngle_interp.(prefix).Talonavicular(:,2), 'color', 'm', 'LineWidth', 2, 'LineStyle', '-');
    xlabel('% Stance Phase');
    ylabel('Joint Angle [{\circ}]');
    title('Eversion (+) / Inversion (-)');
    set(gca, 'Color', 'w');
    set(gca, 'box', 'off');
    
    % Subplot 3: External Rotation/Internal Rotation
    subplot(1, 3, 3);  
    plot(Norm_RelAngle_interp.(prefix).Talonavicular(:,1), 'color', 'm', 'LineWidth', 2, 'LineStyle', '-');
    xlabel('% Stance Phase');
    ylabel('Joint Angle [{\circ}]');
    title('External Rotation (+) / Internal Rotation(-)');
    set(gca, 'Color', 'w');
    set(gca, 'box', 'off');
    set(gcf, 'Position', [10 10 900 300]);
    
    % Save Talonavicular Joint Angles Figure
    saveas(gcf, fullfile(resultsFolder, sprintf('Trial%s_Talonavicular_Joint_Angles.png', trialNumStr)));

end
%%  Calcaneocuboid Graphs
if jointSelection >= 4
    % Plot and Save the joint angle and translation plots
    
    % Joint Angle
    figure('Name','Calcaneocuboid Joint Angles');
    sgtitle('Calcaneocuboid Joint Angles');
    
    % Subplot 1: Dorsiflexion/Plantarflexion
    subplot(1, 3, 1);  
    plot(Norm_RelAngle_interp.(prefix).Calcaneocuboid(:,3), 'color', 'b', 'LineWidth', 2, 'LineStyle', '-');
    xlabel('% Stance Phase');
    ylabel('Joint Angle [{\circ}]');
    title('Dorsiflexion (+) / Plantarflexion (-)');
    set(gca, 'Color', 'w');
    set(gca, 'box', 'off');
    
    % Subplot 2: Eversion/Inversion
    subplot(1, 3, 2);  
    plot(Norm_RelAngle_interp.(prefix).Calcaneocuboid(:,2), 'color', 'b', 'LineWidth', 2, 'LineStyle', '-');
    xlabel('% Stance Phase');
    ylabel('Joint Angle [{\circ}]');
    title('Eversion (+) / Inversion (-)');
    set(gca, 'Color', 'w');
    set(gca, 'box', 'off');
    
    % Subplot 3: External Rotation/Internal Rotation
    subplot(1, 3, 3);  
    plot(Norm_RelAngle_interp.(prefix).Calcaneocuboid(:,1), 'color', 'b', 'LineWidth', 2, 'LineStyle', '-');
    xlabel('% Stance Phase');
    ylabel('Joint Angle [{\circ}]');
    title('External Rotation (+) / Internal Rotation(-)');
    set(gca, 'Color', 'w');
    set(gca, 'box', 'off');
    set(gcf, 'Position', [10 10 900 300]);
    
    % Save Calcaneocuboid Joint Angles Figure
    saveas(gcf, fullfile(resultsFolder, sprintf('Trial%s_Calcaneocuboid_Joint_Angles.png', trialNumStr)));
end 
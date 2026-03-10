%% =========================================================
% STEP 0: Load CT Meshes & Define Coordinate Systems
% =========================================================
% 26 February 2026 - Emily Smith
% Run this script first to load CT and DF data and align in DF space.
% Ensure that the participant variable is changed and corresponds to the
% proper participant name (must be within the parent folder)

clear; close all; clc;

%% ---------------------------------------------------------
% Parent project directory
% ---------------------------------------------------------

parent_path = "/Users/emilysmith/Library/CloudStorage/OneDrive-UniversityofCalgary/PhD/Projects/DEA_Development/MATLAB/MockDEA+CT/Version_2/Test";

%% ---------------------------------------------------------
% Select participant

participant = "CNTRL_006";   % Change
trial_type  = "Neutral";   % Neutral, Walking, etc.
side        = "R";         % R or L


participant_path = fullfile(parent_path, participant);

if ~isfolder(participant_path)
    error('Participant folder not found.');
end

fprintf('Running participant: %s\n', participant);

%% ---------------------------------------------------------
% File Naming
% ---------------------------------------------------------

tibia_stl = participant + "_AUT_Tibia.stl";
talus_stl = participant + "_AUT_Talus.stl";

tibia_tra = participant + "_" + trial_type + "_" + side + "_Tibia.tra";
talus_tra = participant + "_" + trial_type + "_" + side + "_Talus.tra";

tibia_cs_file = participant + "_Tibia_TT_CS.txt";
talus_cs_file = participant + "_Talus_TT_CS.txt";

%% ---------------------------------------------------------
% Load anatomical landmark files (4x3)
% Format: [Origin; Y; Z; X]
% ---------------------------------------------------------

tibia_anat_pts = readmatrix( ...
    fullfile(participant_path, tibia_cs_file));

talus_anat_pts = readmatrix( ...
    fullfile(participant_path, talus_cs_file));

if ~isequal(size(tibia_anat_pts), [4 3])
    error('Tibia CS file must be 4x3');
end

if ~isequal(size(talus_anat_pts), [4 3])
    error('Talus CS file must be 4x3');
end
%% ---------------------------------------------------------
% Read tracking transformation matrices (.tra)
% ---------------------------------------------------------

% Tibia
fid = fopen(fullfile(participant_path, tibia_tra), 'r');
if fid == -1
    error('Cannot open tibia .tra file');
end
line = fgetl(fid);
fclose(fid);

tracking_tibia = reshape( ...
    str2double(strsplit(line,',')), 4, 4)';

% Talus
fid = fopen(fullfile(participant_path, talus_tra), 'r');
if fid == -1
    error('Cannot open talus .tra file');
end
line = fgetl(fid);
fclose(fid);

tracking_talus = reshape( ...
    str2double(strsplit(line,',')), 4, 4)';

%% ---------------------------------------------------------
% Coordinate system transformation
% STL: Y = superior
% Tracking: Z = superior
% ---------------------------------------------------------

coord_transform = [1  0  0  0;
                   0  0  1  0;
                   0  1  0  0;
                   0  0  0  1];

tform_tibia_track = coord_transform * tracking_tibia * inv(coord_transform);
tform_talus_track = coord_transform * tracking_talus * inv(coord_transform);

rot_tibia    = tform_tibia_track(1:3,1:3);
trlate_tibia = tform_tibia_track(1:3,4);

rot_talus    = tform_talus_track(1:3,1:3);
trlate_talus = tform_talus_track(1:3,4);

%% ---------------------------------------------------------
% Load STL meshes
% ---------------------------------------------------------

tibia_mesh = stlread(fullfile(participant_path, tibia_stl));
talus_mesh = stlread(fullfile(participant_path, talus_stl));

tibia_V = tibia_mesh.Points;
tibia_F = tibia_mesh.ConnectivityList;

talus_V = talus_mesh.Points;
talus_F = talus_mesh.ConnectivityList;

%% ---------------------------------------------------------
% Apply STL → tracking axis swap (3x3 only)
% ---------------------------------------------------------

coord_transform_3x3 = coord_transform(1:3,1:3);

tibia_V = (coord_transform_3x3 * tibia_V')';
talus_V = (coord_transform_3x3 * talus_V')';

tibia_anat_pts = (coord_transform_3x3 * tibia_anat_pts')';
talus_anat_pts = (coord_transform_3x3 * talus_anat_pts')';

%% ---------------------------------------------------------
% Apply rigid body transforms
% v_new = R * v_old + t
% ---------------------------------------------------------

tibia_V = (rot_tibia * tibia_V')' + trlate_tibia';
talus_V = (rot_talus * talus_V')' + trlate_talus';

tibia_anat_pts = (rot_tibia * tibia_anat_pts')' + trlate_tibia';
talus_anat_pts = (rot_talus * talus_anat_pts')' + trlate_talus';

%% ---------------------------------------------------------
% Construct anatomical coordinate systems
% ---------------------------------------------------------

[~, tibia_origin, tibia_x, tibia_y, tibia_z] = ...
    LocalCoordinates(tibia_anat_pts);

[~, talus_origin, talus_x, talus_y, talus_z] = ...
    LocalCoordinates(talus_anat_pts);

tibia_axes = [tibia_x', tibia_y', tibia_z'];
talus_axes = [talus_x', talus_y', talus_z'];

%% ---------------------------------------------------------
% Visualization – Tracking Frame
% ---------------------------------------------------------

axis_len = 50;

neutral_fig = figure('Name','Neutral Trial - Global Tracking Coordinates','Color','w');
hold on; axis equal; grid on; view(3);

patch('Faces',tibia_F,'Vertices',tibia_V,...
      'FaceColor',[0.8 0.8 0.9],'EdgeColor','none','FaceAlpha',0.9);

patch('Faces',talus_F,'Vertices',talus_V,...
      'FaceColor',[0.9 0.8 0.7],'EdgeColor','none','FaceAlpha',0.9);

origin_tibia = mean(tibia_V,1);
origin_talus = mean(talus_V,1);

plotCoordinateSystem(origin_tibia, eye(3), axis_len, 'Tibia Track');
plotCoordinateSystem(origin_talus, eye(3), axis_len, 'Talus Track');

xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('Neutral Trial: Global Tracking Coordinates');
camlight headlight;
lighting gouraud;
material dull;
rotate3d on;

%% ---------------------------------------------------------
% Visualization – Anatomical Frame
% ---------------------------------------------------------

figure('Name','Neutral Trial - Anatomical Coordinates','Color','w');
hold on; axis equal; grid on; view(3);

patch('Faces',tibia_F,'Vertices',tibia_V,...
      'FaceColor',[0.8 0.8 0.9],'EdgeColor','none','FaceAlpha',0.9);

patch('Faces',talus_F,'Vertices',talus_V,...
      'FaceColor',[0.9 0.8 0.7],'EdgeColor','none','FaceAlpha',0.9);

plotCoordinateSystem(tibia_origin, tibia_axes, axis_len, 'Tibia Anat');
plotCoordinateSystem(talus_origin, talus_axes, axis_len, 'Talus Anat');

plot3(tibia_anat_pts(:,1),tibia_anat_pts(:,2),tibia_anat_pts(:,3),...
      'ro','MarkerSize',8,'MarkerFaceColor','r');

plot3(talus_anat_pts(:,1),talus_anat_pts(:,2),talus_anat_pts(:,3),...
      'mo','MarkerSize',8,'MarkerFaceColor','m');

xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('Neutral Trial: Anatomical Coordinate Systems');
camlight headlight;
lighting gouraud;
material dull;
rotate3d on;

%% ---------------------------------------------------------
% Save Step 0 Output
% ---------------------------------------------------------

output_filename = participant + "_step_00.mat";

save(fullfile(participant_path, output_filename), ...
    'tibia_V','tibia_F','talus_V','talus_F', ...
    'parent_path','participant_path','participant');

figname = fullfile(participant_path, participant + "_neutral_fig_00.png");
exportgraphics(neutral_fig, figname, 'Resolution', 300);

fprintf('Saved: %s\n', output_filename);
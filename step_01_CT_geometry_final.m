%% =========================================================
% STEP 1: Load CT meshes & define cartilage regions
% Talus = fixed (bone_1)
% Tibia = moving (bone_2)
% =========================================================
% 26 February 2026 - Emily Smith
% Prepare CT bone geometry for DEA by defining:
%   Where cartilage exist
%   How thick it is
%   Where springs will attach

clc; clear; close all;

%% =========================================================
% STEP 1A: Load CT meshes & define cartilage regions
% =========================================================
% Load STL meshes (already transformed to tracking coords)

participant = "CNTRL_006";
parent_path = "/Users/emilysmith/Library/CloudStorage/OneDrive-UniversityofCalgary/PhD/Projects/DEA_Development/MATLAB/MockDEA+CT/Version_2/Test"; 
participant_path = fullfile(parent_path, participant);

step0_file = participant + "_step_00.mat";

load(fullfile(participant_path, step0_file));

% Should contain:
    % tibia_vertices_transformed
    % tibia_faces
    % talus_vertices_transformed
    % talus_faces

%% ---------------------------------------------------------
% Assign bones
% ---------------------------------------------------------

bone_1.mesh.vertices = talus_V;
bone_1.mesh.faces    = talus_F;

bone_2.mesh.vertices = tibia_V;
bone_2.mesh.faces    = tibia_F;

% bone_1.mesh.vertices = talus_vertices_transformed;   % fixed
% bone_1.mesh.faces    = talus_faces;
% 
% bone_2.mesh.vertices = tibia_vertices_transformed;   % moving
% bone_2.mesh.faces    = tibia_faces;

%% ---------------------------------------------------------
% Compute face normals
% ---------------------------------------------------------
bone_1.mesh.face_normals = compute_face_normals( ...
    bone_1.mesh.vertices, bone_1.mesh.faces);

bone_2.mesh.face_normals = compute_face_normals( ...
    bone_2.mesh.vertices, bone_2.mesh.faces);

% ---------------------------------------------------------
% Compute vertex normals (area-weighted)
% ---------------------------------------------------------
bone_1.mesh.vertex_normals = compute_vertex_normals( ...
    bone_1.mesh.vertices, ...
    bone_1.mesh.faces, ...
    bone_1.mesh.face_normals);

bone_2.mesh.vertex_normals = compute_vertex_normals( ...
    bone_2.mesh.vertices, ...
    bone_2.mesh.faces, ...
    bone_2.mesh.face_normals);

%% ---------------------------------------------------------
%  Normal Orientation 
% ---------------------------------------------------------
% Checks if normal points away from bone centre (outward) and assign if
% needed

% ---- Fix talus normal orientation ----
centroid_talus = mean(bone_1.mesh.vertices,1);
vec_test = bone_1.mesh.vertices - centroid_talus;
dot_test = sum(vec_test .* bone_1.mesh.vertex_normals, 2);

if mean(dot_test) < 0
    bone_1.mesh.vertex_normals = -bone_1.mesh.vertex_normals;
    disp('Talus normals flipped.');
end

% ---- Fix tibia normal orientation ----
centroid_tibia = mean(bone_2.mesh.vertices,1);
vec_test = bone_2.mesh.vertices - centroid_tibia;
dot_test = sum(vec_test .* bone_2.mesh.vertex_normals, 2);

if mean(dot_test) < 0
    bone_2.mesh.vertex_normals = -bone_2.mesh.vertex_normals;
    disp('Tibia normals flipped.');
end


%% =========================================================
% STEP 1B: Define articular cartilage regions
% =========================================================
% Compute threshold for which angle of normal can be at to be included in
% the cartilage area. +Z is superior anatomically

% Option A) bottom 10% 
    % z_limit = prctile(bone_2.mesh.vertices(:,3), 10);  % bottom 10%
    % tibia_mask = (bone_2.mesh.vertex_normals(:,3) < -0.8) & ...
    %              (bone_2.mesh.vertices(:,3) < z_limit);
% FUTURE DEVELOPMENT: Calculate threshold based on the angle of the intersection of the
% opposing normals set to a certain angle threshold

% Option B) Set threshold

% Talus dome (superior surface)
talus_mask = bone_1.mesh.vertex_normals(:,3) > 0.85; %0.85

% Tibial plafond (inferior surface)
tibia_mask = bone_2.mesh.vertex_normals(:,3) < -0.85; %-0.85

fprintf('Talus cartilage nodes: %d\n', nnz(talus_mask));
fprintf('Tibia cartilage nodes: %d\n', nnz(tibia_mask));

%% =========================================================
% STEP 1C: Project cartilage surfaces (1.5 mm each)
% =========================================================

cart_thickness_talus = 1.5;   % mm
cart_thickness_tibia = 1.5;   % mm

% Talus cartilage
bone_1.cartilage.vertices = ...
    bone_1.mesh.vertices(talus_mask,:) + ...
    cart_thickness_talus .* ...
    bone_1.mesh.vertex_normals(talus_mask,:);

bone_1.cartilage.normals = ...
    bone_1.mesh.vertex_normals(talus_mask,:);

bone_1.cartilage.thickness = cart_thickness_talus;

% Tibia cartilage
bone_2.cartilage.vertices = ...
    bone_2.mesh.vertices(tibia_mask,:) + ...
    cart_thickness_tibia .* ...
    bone_2.mesh.vertex_normals(tibia_mask,:);

bone_2.cartilage.normals = ...
    bone_2.mesh.vertex_normals(tibia_mask,:);

bone_2.cartilage.thickness = cart_thickness_tibia;

%% =========================================================
% Visualization: check cartilage regions
% =========================================================

both_fig = figure('Position', [1000, 1000, 500, 500], ...
    'Color','w'); hold on; axis equal; grid on;
view(0,0);
zlim([10,70]);
camlight headlight; lighting gouraud;

% Talus mesh
patch('Vertices', bone_1.mesh.vertices, ...
      'Faces', bone_1.mesh.faces, ...
      'FaceColor', [0.6 0.6 0.9], ...
      'EdgeColor', 'none', ...
      'FaceAlpha', 0.3);

% Tibia mesh
patch('Vertices', bone_2.mesh.vertices, ...
      'Faces', bone_2.mesh.faces, ...
      'FaceColor', [0.9 0.6 0.6], ...
      'EdgeColor', 'none', ...
      'FaceAlpha', 0.3);

% Talus cartilage points
scatter3( ...
    bone_1.cartilage.vertices(:,1), ...
    bone_1.cartilage.vertices(:,2), ...
    bone_1.cartilage.vertices(:,3), ...
    8, 'b', 'filled');

% Tibia cartilage points
scatter3( ...
    bone_2.cartilage.vertices(:,1), ...
    bone_2.cartilage.vertices(:,2), ...
    bone_2.cartilage.vertices(:,3), ...
    8, 'r', 'filled');

title('CT-Based Cartilage Regions (1.5 mm Projections)');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');


%% =========================================================
% Talus Only Visualization
% =========================================================

talus_fig = figure('Position', [1000, 1000, 500, 500], ...
    'Name','Talus Cartilage','Color','w'); 
hold on; axis equal; grid on; 
view(3);
%view(90,0); %sag view
camlight headlight; lighting gouraud;

% Talus mesh
patch('Vertices', bone_1.mesh.vertices, ...
      'Faces', bone_1.mesh.faces, ...
      'FaceColor', [0.7 0.7 0.9], ...
      'EdgeColor', 'none', ...
      'FaceAlpha', 0.35);

% Talus cartilage
scatter3( ...
    bone_1.cartilage.vertices(:,1), ...
    bone_1.cartilage.vertices(:,2), ...
    bone_1.cartilage.vertices(:,3), ...
    10, 'b', 'filled');

title(sprintf('Talus Cartilage (n = %d)', ...
    size(bone_1.cartilage.vertices,1)));

xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');

%% =========================================================
% Tibia Only Visualization
% =========================================================

tibia_fig = figure('Position', [1000, 1000, 500, 500], ...
    'Name','Tibia Cartilage','Color','w'); 
hold on; axis equal; grid on; 
view(3); %view(90,0);
%zlim([20,60]);
%ylim([20, 80]);
camlight headlight; lighting gouraud;

% Tibia mesh
patch('Vertices', bone_2.mesh.vertices, ...
      'Faces', bone_2.mesh.faces, ...
      'FaceColor', [0.9 0.7 0.7], ...
      'EdgeColor', 'none', ...
      'FaceAlpha', 0.35);

% Tibia cartilage
scatter3( ...
    bone_2.cartilage.vertices(:,1), ...
    bone_2.cartilage.vertices(:,2), ...
    bone_2.cartilage.vertices(:,3), ...
    10, 'r', 'filled');

title(sprintf('Tibia Cartilage (n = %d)', ...
    size(bone_2.cartilage.vertices,1)));

xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');

%% ---------------------------------------------------------
% Save Step 01 Output (participant-specific, child folder)
% ---------------------------------------------------------

output_mat = fullfile(participant_path, ...
    participant + "_step_01.mat");

output_fig_tibia = fullfile(participant_path, ...
    participant + "_figure_step_01_tibia.png");

output_fig_talus = fullfile(participant_path, ...
    participant + "_figure_step_01_talus.png");

output_fig_both = fullfile(participant_path, ...
    participant + "_figure_step_01_both.png");

% Save data
save(output_mat, 'bone_1', 'bone_2');

% Save figures
exportgraphics(tibia_fig, output_fig_tibia, 'Resolution', 300);
exportgraphics(talus_fig, output_fig_talus, 'Resolution', 300);
exportgraphics(both_fig, output_fig_both, 'Resolution', 300);

fprintf("Saved Step 01 outputs for %s\n", participant);
%% =========================================================
% Local functions
% =========================================================

% Face Normals
function face_normals = compute_face_normals(vertices, faces)

    v1 = vertices(faces(:,1),:);
    v2 = vertices(faces(:,2),:);
    v3 = vertices(faces(:,3),:);

    % Unnormalized face normals (area-weighted)
    face_normals = cross(v2 - v1, v3 - v1, 2);
end

% Vertex Normals
function vertex_normals = compute_vertex_normals(vertices, faces, face_normals)

    n_vertices = size(vertices,1);
    vertex_normals = zeros(n_vertices,3);

    for f = 1:size(faces,1)
        for k = 1:3
            v_idx = faces(f,k);
            vertex_normals(v_idx,:) = ...
                vertex_normals(v_idx,:) + face_normals(f,:);
        end
    end

    % Normalize to unit length
    vertex_normals = vertex_normals ./ vecnorm(vertex_normals,2,2);
end

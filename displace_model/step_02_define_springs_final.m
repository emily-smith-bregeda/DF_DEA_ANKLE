%% =========================================================
% STEP 2: Define DEA springs on talus cartilage
% =========================================================
% 26 February 2026 - Emily Smith
% Converts the cartilage surface into a distributed system of 
% compressive springs with defined stiffness and area.
% Enables contact force and stress computation
clc; clear; close all;

%% ---------------------------------------------------------
% Load Step 01 output
% ---------------------------------------------------------
participant = "CNTRL_006";   % Change
parent_path = "/Users/emilysmith/Library/CloudStorage/OneDrive-UniversityofCalgary/PhD/Projects/DEA_Development/MATLAB/MockDEA+CT/Version_2/Test";
participant_path = fullfile(parent_path, participant);

step1_file = participant + "_step_01.mat";

load(fullfile(participant_path, step1_file));

%% ---------------------------------------------------------
% Undeformed length
% ---------------------------------------------------------
length_0 = bone_1.cartilage.thickness + bone_2.cartilage.thickness;
dea.cartilage.length_0 = length_0;

%% ---------------------------------------------------------
% Material properties
% ---------------------------------------------------------
E_cart = 12;       % MPa
nu_cart = 0.42;

poisson_factor = (1 - nu_cart) / ...
    ((1 - 2*nu_cart) * (1 + nu_cart));

%% =========================================================
% Compute Nodal Area (ONLY on talus cartilage region)
% =========================================================

% Full triangle areas
tri_areas = triangle_areas( ...
    bone_1.mesh.vertices, bone_1.mesh.faces);

% Identify triangles fully belonging to cartilage region
talus_mask = ismember( ...
    1:size(bone_1.mesh.vertices,1), ...
    find(ismember(bone_1.mesh.vertices, ...
    bone_1.mesh.vertices(bone_1.mesh.vertex_normals(:,3) > 0.85,:), 'rows')));

% Instead of recomputing mask logic, better:
talus_vertex_mask = false(size(bone_1.mesh.vertices,1),1);
talus_vertex_mask( ...
    bone_1.mesh.vertex_normals(:,3) > 0.85) = true;

% Triangles where all 3 vertices are in cartilage region
faces = bone_1.mesh.faces;
tri_mask = talus_vertex_mask(faces(:,1)) & ...
           talus_vertex_mask(faces(:,2)) & ...
           talus_vertex_mask(faces(:,3));

cartilage_faces = faces(tri_mask,:);
cartilage_tri_areas = tri_areas(tri_mask);

%% ---------------------------------------------------------
% Distribute triangle area to nodes (1/3 per vertex)
% ---------------------------------------------------------
n_cart_nodes = size(bone_1.cartilage.vertices,1);
nodal_area = zeros(size(bone_1.mesh.vertices,1),1);

for i = 1:size(cartilage_faces,1)
    tri = cartilage_faces(i,:);
    area = cartilage_tri_areas(i) / 3;
    nodal_area(tri) = nodal_area(tri) + area;
end

% Keep only cartilage nodes
cart_indices = find(talus_vertex_mask);
A_i = nodal_area(cart_indices);

%% =========================================================
% Define Springs
% =========================================================

dea.springs.vertices = bone_1.cartilage.vertices;
dea.springs.normals  = bone_1.cartilage.normals;

dea.springs.area = A_i;

dea.springs.k = ...
    E_cart * poisson_factor .* (A_i ./ length_0);

n_springs = numel(A_i);

dea.springs.compression = zeros(n_springs,1);
dea.springs.force       = zeros(n_springs,1);
dea.springs.active      = false(n_springs,1);

%% ---------------------------------------------------------
% Save Step 02 Output
% ---------------------------------------------------------

output_step2_file = fullfile(participant_path, ...
    participant + "_step_02.mat");

save(output_step2_file, ...
    'bone_1', 'bone_2', 'dea');

fprintf('Saved: %s\n', participant + "_step_02_springs_CT.mat");
fprintf('Springs defined: %d\n', n_springs);
fprintf('Mean nodal area: %.3f mm^2\n', mean(A_i));

fprintf("Saved Step 02 outputs for %s\n", participant);

%% =========================================================
% Local function
% =========================================================

function areas = triangle_areas(vertices, faces)

    v1 = vertices(faces(:,1),:);
    v2 = vertices(faces(:,2),:);
    v3 = vertices(faces(:,3),:);

    areas = 0.5 * vecnorm(cross(v2 - v1, v3 - v1, 2), 2, 2);
end
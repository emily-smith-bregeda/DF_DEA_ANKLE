%% =========================================================
% STEP 4: Mesh-Based Gap & Compression (CT)
% DEA-consistent KD-tree implementation
%% =========================================================
% 26 February 2026 - Emily Smith
clc; clear; close all;

participant = "CNTRL_006";   % Change
parent_path = "/Users/emilysmith/Library/CloudStorage/OneDrive-UniversityofCalgary/PhD/Projects/DEA_Development/MATLAB/MockDEA+CT/Version_2/Test";
participant_path = fullfile(parent_path, participant);

%% ---------------------------------------------------------
% Load Step 03 output
% ---------------------------------------------------------

step3_file = participant + "_step_03.mat";

load(fullfile(participant_path, step3_file));

%% ---------------------------------------------------------
% Load displacement + springs
% ---------------------------------------------------------

cart_1_pts = bone_1.cartilage.vertices;
normals_1  = bone_1.cartilage.normals;

n_points = size(cart_1_pts,1);
n_frames = numel(motion);

length_0 = dea.cartilage.length_0;

%% ---------------------------------------------------------
% Preallocate
% ---------------------------------------------------------

compression = zeros(n_points, n_frames);

%% =========================================================
% Loop Over Frames
% =========================================================
% KD-tree used to accelerate nearest triangle search
% Expected runtime: ~2–6 minutes for 100 frames

%tic
for f = 1:n_frames

    fprintf('Processing frame %d / %d\n', f, n_frames);

    % -----------------------------------------------------
    % Moving tibia mesh
    % -----------------------------------------------------

    V = motion(f).bone_2_vertices;
    F = bone_2.mesh.faces;

    % -----------------------------------------------------
    % Triangle vertices
    % -----------------------------------------------------

    V1 = V(F(:,1),:);
    V2 = V(F(:,2),:);
    V3 = V(F(:,3),:);

    % -----------------------------------------------------
    % Triangle centroids (for KD-tree)
    % -----------------------------------------------------

    tri_centroids = (V1 + V2 + V3)/3;

    % -----------------------------------------------------
    % KD-tree nearest triangle search (vectorized)
    % -----------------------------------------------------

    idx_all = knnsearch( ...
        tri_centroids, ...
        cart_1_pts, ...
        'K',30, ...
        'NSMethod','kdtree');

    % -----------------------------------------------------
    % Compute compression
    % -----------------------------------------------------

    compression_frame = zeros(n_points,1);

    for i = 1:n_points

        p = cart_1_pts(i,:);
        idx = idx_all(i,:);

        min_dist = inf;
        cp_best = [0 0 0];

        for j = 1:30

            a = V1(idx(j),:);
            b = V2(idx(j),:);
            c = V3(idx(j),:);

            cp = closestPointOnTriangleFull(p,a,b,c);

            d = norm(cp - p);

            if d < min_dist
                min_dist = d;
                cp_best = cp;
            end

        end

        % Normal-direction gap (DEA definition)

        gap = dot(cp_best - p, normals_1(i,:));

        % Compression = penetration only

        compression_frame(i) = max(0, -gap);

    end

    compression(:,f) = compression_frame;

end
%toc

dea.contact.compression = compression;

fprintf('Final max compression: %.4f mm\n', max(compression(:)));

%% =========================================================
% Visualization
% =========================================================

viz_fig = figure ('Position', [500 1000 600 700], 'Color', 'w'); 
%figure('WindowState','fullscreen', 'Color','w');

hold on;
axis equal;
grid on;
view(135, 25)

xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');

camlight headlight;
lighting gouraud;

% --- Talus mesh (fixed)

patch( ...
    'Vertices', bone_1.mesh.vertices, ...
    'Faces', bone_1.mesh.faces, ...
    'FaceColor', [0.6 0.6 0.9], ...
    'EdgeColor', 'none', ...
    'FaceAlpha', 0.35);

% --- Tibia mesh (moving)

patch_bone_2 = patch( ...
    'Vertices', motion(1).bone_2_vertices, ...
    'Faces', bone_2.mesh.faces, ...
    'FaceColor', [0.9 0.6 0.6], ...
    'EdgeColor', 'none', ...
    'FaceAlpha', 0.25);

% --- Compression scatter

sc = scatter3( ...
    cart_1_pts(:,1), ...
    cart_1_pts(:,2), ...
    cart_1_pts(:,3), ...
    20, ...
    compression(:,1), ...
    'filled');

colormap("sky"); %jet = rainbow

max_comp = max(compression(:));

if max_comp <= 0
    max_comp = 1e-6;
end

caxis([0 max_comp]);

cb = colorbar;
cb.Label.String = 'Compression (mm)';

title('CT-Based DEA Compression');

drawnow;

% ---------------------------------------------------------
% Animate
% ---------------------------------------------------------

for f = 1:n_frames

    patch_bone_2.Vertices = motion(f).bone_2_vertices;

    sc.CData = compression(:,f);

    title(sprintf( ...
        'Compression (mm) — Frame %d / %d', ...
        f, n_frames));

    drawnow;
    pause(0.05);

end

%% =========================================================
% Save GIF
% =========================================================

gif_name = participant + "_gif_step_04_CT_compression.gif";
gif_path = fullfile(participant_path, gif_name);

delay_time = 0.05;

fprintf('Saving GIF...\n');

figure(viz_fig);

for f = 1:n_frames

    patch_bone_2.Vertices = motion(f).bone_2_vertices;

    sc.CData = compression(:,f);

    drawnow;

    frame = getframe(viz_fig);
    img = frame2im(frame);
    [imind, cm] = rgb2ind(img, 256);

    if f == 1

        imwrite( ...
            imind, cm, gif_path, ...
            'gif', ...
            'Loopcount', inf, ...
            'DelayTime', delay_time);

    else

        imwrite( ...
            imind, cm, gif_path, ...
            'gif', ...
            'WriteMode', 'append', ...
            'DelayTime', delay_time);

    end

end

fprintf('GIF saved: %s\n', gif_path);

%% =========================================================
% Talus cartilage animation (point cloud version)
% =========================================================

viz_fig = figure('Position', [500 1000 600 700], ...
    'Color','w', ...
    'InvertHardcopy','off');

hold on;
axis equal;
axis vis3d;
grid on;

view(0,90);   % superior view

xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');

camlight headlight;
lighting gouraud;

% Scatter plot of cartilage nodes

sc = scatter3( ...
    bone_1.cartilage.vertices(:,1), ...
    bone_1.cartilage.vertices(:,2), ...
    bone_1.cartilage.vertices(:,3), ...
    25, ...
    compression(:,1), ...
    'filled');

colormap("cool");

max_comp = max(compression(:));
if max_comp <= 0
    max_comp = 1e-6;
end

caxis([0 max_comp]);

cb = colorbar;
cb.Label.String = 'Compression (mm)';

title('Talus Cartilage — Superior View');

drawnow;

% Animate compression

for f = 1:n_frames

    sc.CData = compression(:,f);

    title(sprintf( ...
        'Talus Cartilage Compression — Frame %d / %d', ...
        f, n_frames));

    drawnow;
    pause(0.05);

end

%% =========================================================
% Save GIF — Talus cartilage compression
% =========================================================

gif_name   = participant+ '_gif_step_04_talus_cartilage_superior.gif';
delay_time = 0.05;

fprintf('Saving GIF...\n');

% Ensure white background in saved frames
set(viz_fig,'Color','w');
set(gca,'Color','w');

for f = 1:n_frames

    % Update compression colors
    sc.CData = compression(:,f);

    title(sprintf( ...
        'Talus Cartilage Compression — Frame %d / %d', ...
        f, n_frames));

    drawnow;

    % Capture frame
    frame = getframe(viz_fig);
    img   = frame2im(frame);

    % Convert to indexed image
    [imind, cm] = rgb2ind(img, 256);

    % Write GIF
    if f == 1

        imwrite(imind, cm, gif_path, ...
            'gif', ...
            'Loopcount', inf, ...
            'DelayTime', delay_time);

    else

        imwrite(imind, cm, gif_path, ...
            'gif', ...
            'WriteMode', 'append', ...
            'DelayTime', delay_time);

    end

end

fprintf('GIF saved: %s\n', gif_name);

% Final Frame export
final_frame_name = participant + '_step_04_talus_cartilage_superior_final.png';

% Set last frame explicitly
sc.CData = compression(:,end);

title(sprintf( ...
    'Talus Cartilage Compression — Frame %d / %d', ...
    n_frames, n_frames), ...
    'Color','k');

drawnow;

% Save high-quality image
final_frame_name = participant + "_final_frame_step_04_CT_compression.png";
final_frame_path = fullfile(participant_path, final_frame_name);

exportgraphics(viz_fig, final_frame_path, ...
    'Resolution',300, ...
    'BackgroundColor','white');

fprintf('Final frame saved: %s\n', final_frame_path);
fprintf('Final frame saved: %s\n', final_frame_name);

%% =========================================================
% Save Step 04 Output (participant-specific)
% =========================================================

output_step4_file = fullfile(participant_path, ...
    participant + "_step_04.mat");

save(output_step4_file, ...
    'bone_1', ...
    'bone_2', ...
    'motion', ...
    'dea');

fprintf('Saved: %s\n', participant + "_step_04.mat");
fprintf('STEP 4 complete: CT-based compression computed.\n');

%% =========================================================
% Helper Function
% =========================================================

function cp = closestPointOnTriangleFull(p, a, b, c)

ab = b - a;
ac = c - a;
ap = p - a;

d1 = dot(ab, ap);
d2 = dot(ac, ap);

if d1 <= 0 && d2 <= 0
    cp = a;
    return;
end

bp = p - b;
d3 = dot(ab, bp);
d4 = dot(ac, bp);

if d3 >= 0 && d4 <= d3
    cp = b;
    return;
end

vc = d1*d4 - d3*d2;

if vc <= 0 && d1 >= 0 && d3 <= 0
    v = d1 / (d1 - d3);
    cp = a + v * ab;
    return;
end

cp_vec = p - c;
d5 = dot(ab, cp_vec);
d6 = dot(ac, cp_vec);

if d6 >= 0 && d5 <= d6
    cp = c;
    return;
end

vb = d5*d2 - d1*d6;

if vb <= 0 && d2 >= 0 && d6 <= 0
    w = d2 / (d2 - d6);
    cp = a + w * ac;
    return;
end

va = d3*d6 - d5*d4;

if va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0
    w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    cp = b + w * (c - b);
    return;
end

denom = 1 / (va + vb + vc);
v = vb * denom;
w = vc * denom;

cp = a + ab * v + ac * w;

end
%% =========================================================
% STEP 5: Compute Forces, Stress & Contact Area (CT)
% =========================================================
% 26 February 2026 - Emily Smith
%% =========================================================
clc; clear; close all;

participant = "CNTRL_006";   % Change
parent_path = "/Users/emilysmith/Library/CloudStorage/OneDrive-UniversityofCalgary/PhD/Projects/DEA_Development/MATLAB/MockDEA+CT/Version_2/Test";
participant_path = fullfile(parent_path, participant);

%% ---------------------------------------------------------
% Load Step 04 output
% ---------------------------------------------------------

step4_file = participant + "_step_04.mat";

load(fullfile(participant_path, step4_file));

%% ---------------------------------------------------------
% Load penetration results
% ---------------------------------------------------------

compression = dea.contact.compression;

n_points = size(compression,1);
n_frames = size(compression,2);

A_i = dea.springs.area;
k_i = dea.springs.k;

%% ---------------------------------------------------------
% Preallocate
% ---------------------------------------------------------
force_node   = zeros(n_points, n_frames);
stress_node  = zeros(n_points, n_frames);
force_total  = zeros(n_frames,1);
area_contact = zeros(n_frames,1);

%% =========================================================
% Loop over frames
% =========================================================

for f = 1:n_frames

    %fprintf('Processing frame %d / %d\n', f, n_frames);

    delta = compression(:,f);

    % Nodal force
    force_node(:,f) = k_i .* delta;

    % Contact mask
    contact_mask = delta > 0;

    % Nodal stress
    stress_node(contact_mask,f) = ...
        force_node(contact_mask,f) ./ A_i(contact_mask);

    % Integrated force
    force_total(f) = sum(force_node(:,f));

    % Contact area
    area_contact(f) = sum(A_i(contact_mask));
end

%% ---------------------------------------------------------
% Store
% ---------------------------------------------------------
dea.force.node   = force_node;
dea.force.total  = force_total;
dea.stress.node  = stress_node;
dea.area.contact = area_contact;

%% ---------------------------------------------------------
% Quick summaries
% ---------------------------------------------------------

% Contact Force
fprintf('--- Contact Force ---\n')

disp('Expected Peak Contact Force: 3000-6000N')
fprintf('Calculated Peak Contact Force: %.2f N\n', max(force_total));

disp('Expected Mean Contact Force: 1000-3000N')
fprintf('Calculated Mean Contact Force: %.2f N\n', mean(force_total));


% Contact Area
fprintf('--- Contact Area ---\n')

disp('Expected Peak Contact Area: 400-800mm^2');
fprintf('Caclculated Peak Contact Area: %.2f mm^2\n', max(area_contact));

disp('Expected Mean Contact Area: 200-400mm^2');
fprintf('Caclculated Mean Contact Area: %.2f mm^2\n', mean(area_contact));


% Contact Stresss
fprintf('--- Contact Stress ---\n')

disp('Expected Peak Contact Stress: 15-25MPa');
fprintf('Caclculated Peak Stress: %.2f MPa\n', max(stress_node(:)));

disp('Expected Mean Contact Stress: 2-6MPa');
fprintf('Caclculated Mean Stress: %.2f MPa\n', mean(stress_node(:), 'omitnan'));


%% =========================================================
% Stress Animation
% =========================================================

figure('Position',[500 100 850 700], 'Color','w');
hold on; axis equal; grid on;
view(135, 25);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
camlight headlight; lighting gouraud;

patch('Vertices', bone_1.mesh.vertices, ...
      'Faces', bone_1.mesh.faces, ...
      'FaceColor', [0.6 0.6 0.9], ...
      'EdgeColor', 'none', ...
      'FaceAlpha', 0.35);

patch_bone_2 = patch('Vertices', motion(1).bone_2_vertices, ...
      'Faces', bone_2.mesh.faces, ...
      'FaceColor', [0.9 0.6 0.6], ...
      'EdgeColor', 'none', ...
      'FaceAlpha', 0.25);

sc = scatter3( ...
    bone_1.cartilage.vertices(:,1), ...
    bone_1.cartilage.vertices(:,2), ...
    bone_1.cartilage.vertices(:,3), ...
    20, stress_node(:,1), 'filled');

colormap("parula");
caxis([0 max(stress_node(:))]);
colorbar;

for f = 1:n_frames

    patch_bone_2.Vertices = motion(f).bone_2_vertices;
    sc.CData = stress_node(:,f);

    title(sprintf('Contact Stress (MPa) — displacement = %.2f mm', ...
        norm(motion(f).displacement)));

    drawnow;
    pause(0.08);
end


%% ---------------------------------------------------------
% Save Stress Animation as GIF
% ---------------------------------------------------------

gif_name   = participant + '_gif_step_05_CT_contact_stress_animation.gif';
gif_path = fullfile(participant_path, gif_name);
delay_time = 0.08;

set(gcf,'Color','w');

for f = 1:n_frames

    patch_bone_2.Vertices = motion(f).bone_2_vertices;
    sc.CData = stress_node(:,f);

    title(sprintf('Contact Stress (MPa) — displacement = %.2f mm', ...
        norm(motion(f).displacement)));

    drawnow;

    frame = getframe(gcf);
    img = frame2im(frame);
    [imind, cm] = rgb2ind(img, 256);

    if f == 1
        imwrite(imind, cm, gif_path, ...
            'gif', 'Loopcount', inf, ...
            'DelayTime', delay_time);
    else
        imwrite(imind, cm, gif_path, ...
            'gif', 'WriteMode', 'append', ...
            'DelayTime', delay_time);
    end
end

fprintf('Stress animation saved: %s\n', gif_name);


%% =========================================================
% Talus cartilage STRESS animation (point cloud version)
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
    stress_node(:,1), ...
    'filled');

colormap("parula");

max_stress = max(stress_node(:));
if max_stress <= 0
    max_stress = 1e-6;
end

caxis([0 max_stress]);

cb = colorbar;
cb.Label.String = 'Contact Stress (MPa)';
cb.Color = 'k';

title('Talus Cartilage Contact Stress','Color','k');

for f = 1:n_frames

    sc.CData = stress_node(:,f);

    title(sprintf('Contact Stress (MPa) — Frame %d', f), ...
        'Color','k');

    drawnow;

end


% ---------------------------------------------------------
% Save STRESS animation as GIF
% ---------------------------------------------------------

gif_name   = participant + '_gif_step_05_talus_cartilage_stress_superior.gif';
gif_path = fullfile(participant_path, gif_name);
delay_time = 0.08;

for f = 1:n_frames

    sc.CData = stress_node(:,f);

    title(sprintf('Contact Stress (MPa) — Frame %d', f), ...
        'Color','k');

    drawnow;

    frame = getframe(viz_fig);
    img   = frame2im(frame);
    [imind, cm] = rgb2ind(img,256);

    if f == 1
        imwrite(imind, cm, gif_path, ...
            'gif','Loopcount',inf, ...
            'DelayTime',delay_time);
    else
        imwrite(imind, cm, gif_path, ...
            'gif','WriteMode','append', ...
            'DelayTime',delay_time);
    end
end

fprintf('Stress GIF saved: %s\n', gif_name);

%% ---------------------------------------------------------
% First vs Final Frame (Superior View)
% ---------------------------------------------------------

figure('Color','w','Position',[600 1000 1200 600]);

frames_to_plot = [1 n_frames];

for i = 1:2

    subplot(1,2,i);
    hold on;
    axis equal;
    axis vis3d;
    grid on;
    view(0,90);

    scatter3( ...
        bone_1.cartilage.vertices(:,1), ...
        bone_1.cartilage.vertices(:,2), ...
        bone_1.cartilage.vertices(:,3), ...
        25, ...
        stress_node(:,frames_to_plot(i)), ...
        'filled');

    colormap("parula");
    caxis([0 max_stress]);

    xlabel('X (mm)');
    ylabel('Y (mm)');
    zlabel('Z (mm)');

    camlight headlight;
    lighting gouraud;

    if i == 1
        title('First Frame','Color','k');
    else
        title('Final Frame','Color','k');
    end

end

cb = colorbar;
cb.Position = [0.92 0.2 0.02 0.6];
cb.Label.String = 'Contact Stress (MPa)';
cb.Color = 'k';

sgtitle('Talus Cartilage Contact Stress (Superior View)', ...
    'Color','k');

% Save high-res PNG
exportgraphics(gcf, participant + '_step_05_talus_stress_first_vs_last.png','Resolution',600);

fprintf('First vs Final frame figure saved.\n');
%% ---------------------------------------------------------
% Figure 1: Force–Displacement
% ---------------------------------------------------------

displacement_mag = arrayfun(@(m) norm(m.displacement), motion);

force_fig = figure('Color','w');
plot(displacement_mag, force_total, 'LineWidth',2);
xlabel('Displacement (mm)');
ylabel('Total Contact Force (N)');
title(participant + ": CT-Based DEA Force vs Displacement");
grid on;

force_name = participant + "_fig_step_05_CT_force_displacement.png";
force_path = fullfile(participant_path, force_name);

exportgraphics(force_fig, force_path, 'Resolution', 300);

fprintf('Saved: %s\n', force_path);


% ---------------------------------------------------------
% Figure 2: Contact Area–Displacement
% ---------------------------------------------------------

area_fig = figure('Color','w');
plot(displacement_mag, area_contact, 'LineWidth',2);
xlabel('Displacement (mm)');
ylabel('Contact Area (mm^2)');
title(participant + ": CT-Based DEA Contact Area vs Displacement");
grid on;

area_name = participant + "_fig_step_05_CT_contact_area.png";
area_path = fullfile(participant_path, area_name);

exportgraphics(area_fig, area_path, 'Resolution', 300);

fprintf('Saved: %s\n', area_path);


% ---------------------------------------------------------
% Figure 3: Peak Stress–Displacement
% ---------------------------------------------------------

stress_peak = max(stress_node,[],1);

stress_fig = figure('Color','w');
plot(displacement_mag, stress_peak, 'LineWidth',2);
xlabel('Displacement (mm)');
ylabel('Peak Contact Stress (MPa)');
title(participant + ": CT-Based DEA Peak Stress vs Displacement");
grid on;

stress_name = participant + "_fig_step_05_CT_peak_stress.png";
stress_path = fullfile(participant_path, stress_name);

exportgraphics(stress_fig, stress_path, 'Resolution', 300);

fprintf('Saved: %s\n', stress_path);
%% =========================================================
% Save Step 05 Output (participant-specific)
% =========================================================

output_step5_file = fullfile(participant_path, ...
    participant + "_step_05_CT_forces.mat");

save(output_step5_file, ...
    'bone_1', ...
    'bone_2', ...
    'motion', ...
    'dea');

fprintf('Saved: %s\n', participant + "_step_05.mat");
fprintf('STEP 5 complete: CT-based contact forces and stress computed.\n');
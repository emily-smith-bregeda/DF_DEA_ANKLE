%% =========================================================
% STEP 3: Synthetic displacement of tibia (CT-based)
% =========================================================
% 26 February 2026 - Emily Smith
% applies a controlled rigid-body displacement to the tibia to 
% simulate joint compression
clc; clear; close all;

participant = "CNTRL_006";   % Change
parent_path = "/Users/emilysmith/Library/CloudStorage/OneDrive-UniversityofCalgary/PhD/Projects/DEA_Development/MATLAB/MockDEA+CT/Version_2/Test";
participant_path = fullfile(parent_path, participant);

%% ---------------------------------------------------------
% Load Step 02 output
% ---------------------------------------------------------

step2_file = participant + "_step_02.mat";

load(fullfile(participant_path, step2_file));

%% ---------------------------------------------------------
% Displacement parameters
% ---------------------------------------------------------
n_frames = 100;     % 100 average for DF 
disp_max = 1.5;    % mm - 1.5mm considered physiologic

displacements = linspace(0, disp_max, n_frames);

% Move tibia downward (inferior direction = -Z)
disp_dir = [0 0 -1];
disp_dir = disp_dir / norm(disp_dir);

%% ---------------------------------------------------------
% Preallocate motion structure
% ---------------------------------------------------------
motion = struct( ...
    'bone_2_vertices', [], ...
    'bone_2_cartilage', [], ...
    'displacement', []);

motion(n_frames).bone_2_vertices = [];

%% ---------------------------------------------------------
% Loop over frames
% ---------------------------------------------------------
for f = 1:n_frames

    u = displacements(f) * disp_dir;

    motion(f).bone_2_vertices = ...
        bone_2.mesh.vertices + u;

    motion(f).bone_2_cartilage = ...
        bone_2.cartilage.vertices + u;

    motion(f).displacement = u;
end

%% ---------------------------------------------------------
% Animation: CT-Based Synthetic Displacement (100 frames)
% ---------------------------------------------------------

anim_fig = figure('Position',[500 1000 850 700], 'Color','w');
hold on; axis equal; grid on;
view(0,1); %0,1 for joint view
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
zlim([10,70]);
camlight headlight; lighting gouraud;

% Talus (fixed)
patch('Vertices', bone_1.mesh.vertices, ...
      'Faces', bone_1.mesh.faces, ...
      'FaceColor', [0.6 0.6 0.9], ...
      'EdgeColor', 'none', ...
      'FaceAlpha', 0.4);

% Tibia (moving)
patch_bone_2 = patch('Vertices', motion(1).bone_2_vertices, ...
      'Faces', bone_2.mesh.faces, ...
      'FaceColor', [0.9 0.6 0.6], ...
      'EdgeColor', 'none', ...
      'FaceAlpha', 0.4);

title('CT-Based Synthetic Displacement');

drawnow;

for f = 1:n_frames

    patch_bone_2.Vertices = motion(f).bone_2_vertices;

    title(sprintf('Displacement = %.2f mm', ...
        norm(motion(f).displacement)));

    drawnow;
    pause(0.08);   % adjust speed
end


% ---------------------------------------------------------
% Save GIF
% ---------------------------------------------------------

gif_name   = participant + '_gif_step_03_CT_displacement.gif';
delay_time = 0.08;

fprintf('Saving GIF...\n');

figure(anim_fig);
set(anim_fig,'Color','w');
drawnow;

for f = 1:n_frames

    patch_bone_2.Vertices = motion(f).bone_2_vertices;

    title(sprintf('Displacement = %.2f mm', ...
        norm(motion(f).displacement)));

    drawnow;
    pause(0.01);

    frame = getframe(anim_fig);
    img = frame2im(frame);
    [imind, cm] = rgb2ind(img, 256);

    if f == 1
        imwrite(imind, cm, gif_name, ...
            'gif', 'Loopcount', inf, ...
            'DelayTime', delay_time);
    else
        imwrite(imind, cm, gif_name, ...
            'gif', 'WriteMode', 'append', ...
            'DelayTime', delay_time);
    end
end

fprintf('GIF saved: %s\n', gif_name);
%% ---------------------------------------------------------
% Animation: CT Displacement with Cartilage Surfaces
% ---------------------------------------------------------

anim_fig = figure('Position',[1000 1000 850 700], 'Color','w');
hold on; axis equal; grid on;
view(0,1); %view(30,25);
zlim([10,70]);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
camlight headlight; lighting gouraud;

% --- Talus mesh (fixed)
patch('Vertices', bone_1.mesh.vertices, ...
      'Faces', bone_1.mesh.faces, ...
      'FaceColor', [0.6 0.6 0.9], ...
      'EdgeColor', 'none', ...
      'FaceAlpha', 0.35);

% --- Tibia mesh (moving)
patch_bone_2 = patch('Vertices', motion(1).bone_2_vertices, ...
      'Faces', bone_2.mesh.faces, ...
      'FaceColor', [0.9 0.6 0.6], ...
      'EdgeColor', 'none', ...
      'FaceAlpha', 0.35);

% --- Talus cartilage (fixed)
cart_1_scatter = scatter3( ...
    bone_1.cartilage.vertices(:,1), ...
    bone_1.cartilage.vertices(:,2), ...
    bone_1.cartilage.vertices(:,3), ...
    10, 'b', 'filled');

% --- Tibia cartilage (moving)
cart_2_scatter = scatter3( ...
    motion(1).bone_2_cartilage(:,1), ...
    motion(1).bone_2_cartilage(:,2), ...
    motion(1).bone_2_cartilage(:,3), ...
    10, 'r', 'filled');

title('CT-Based Synthetic Displacement (with Cartilage)');
drawnow;

for f = 1:n_frames

    % Update tibia mesh
    patch_bone_2.Vertices = motion(f).bone_2_vertices;

    % Update tibia cartilage
    cart_2_scatter.XData = motion(f).bone_2_cartilage(:,1);
    cart_2_scatter.YData = motion(f).bone_2_cartilage(:,2);
    cart_2_scatter.ZData = motion(f).bone_2_cartilage(:,3);

    title(sprintf('Displacement = %.2f mm', ...
        norm(motion(f).displacement)));

    drawnow;
    pause(0.08);
end

% ---------------------------------------------------------
% Save GIF (Bones + Cartilage)
% ---------------------------------------------------------

gif_name   = participant + '_gif_step_03_CT_displacement_with_cartilage.gif';
delay_time = 0.08;

fprintf('Saving GIF with cartilage...\n');

figure(anim_fig);
set(anim_fig,'Color','w');
drawnow;

for f = 1:n_frames

    patch_bone_2.Vertices = motion(f).bone_2_vertices;

    cart_2_scatter.XData = motion(f).bone_2_cartilage(:,1);
    cart_2_scatter.YData = motion(f).bone_2_cartilage(:,2);
    cart_2_scatter.ZData = motion(f).bone_2_cartilage(:,3);

    title(sprintf('Displacement = %.2f mm', ...
        norm(motion(f).displacement)));

    drawnow;
    pause(0.01);

    frame = getframe(anim_fig);
    img = frame2im(frame);
    [imind, cm] = rgb2ind(img, 256);

    if f == 1
        imwrite(imind, cm, gif_name, ...
            'gif', 'Loopcount', inf, ...
            'DelayTime', delay_time);
    else
        imwrite(imind, cm, gif_name, ...
            'gif', 'WriteMode', 'append', ...
            'DelayTime', delay_time);
    end
end

fprintf('GIF saved: %s\n', gif_name);

%% ---------------------------------------------------------
% Save Step 03 Output (participant-specific)
% ---------------------------------------------------------

output_step3_file = fullfile(participant_path, ...
    participant + "_step_03.mat");

save(output_step3_file, ...
     'bone_1', 'bone_2', 'dea', 'motion');

fprintf('Saved: %s\n', participant + "_step_03.mat");

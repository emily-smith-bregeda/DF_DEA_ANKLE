%% B. Set up local coordinate systems and  (BuildBoneModels.m)

function [Bone_model, Bone_model_Auto, Coordinate, LCS] = BuildBoneModels(folderPath, jointSelection)
% BuildBoneModels - Load bone models, coordinate systems, and transformations

    % Define structures
    Bone_model = struct();
    Bone_model_Auto = struct();
    Coordinate = struct();
    LCS = struct();
    % LCS_Auto = struct();

    % Load models folder
    modelsFolder = fullfile(folderPath, 'Models');

    %% ---- Mandatory bones (Tibia & Talus) ----
    Bone_model.Tibia = stlread(fullfile(modelsFolder, 'AUT_Tibia.stl'));
    Bone_model.Talus = stlread(fullfile(modelsFolder, 'AUT_Talus.stl'));
    Bone_model_Auto.Tibia.Points = [Bone_model.Tibia.Points, ones(size(Bone_model.Tibia.Points,1),1)];
    Bone_model_Auto.Talus.Points = [Bone_model.Talus.Points, ones(size(Bone_model.Talus.Points,1),1)];

    % Tibia & Talus TT coordinate systems
    Coordinate.TibiaTT = load(fullfile(modelsFolder, 'Tibia_TT_CS.txt'));
    Coordinate.TalusTT = load(fullfile(modelsFolder, 'Talus_TT_CS.txt'));

    % Setup transformations (TT)
    % [LCS2Auto.TibiaTT, LCS_Auto.TibiaTT.Origin, LCS_Auto.TibiaTT.X, LCS_Auto.TibiaTT.Y, LCS_Auto.TibiaTT.Z] = LocalCoordinates(Coordinate.TibiaTT);
    % [LCS2Auto.TalusTT, LCS_Auto.TalusTT.Origin, LCS_Auto.TalusTT.X, LCS_Auto.TalusTT.Y, LCS_Auto.TalusTT.Z] = LocalCoordinates(Coordinate.TalusTT);
    [LCS.TibiaTT, ~] = LocalCoordinates(Coordinate.TibiaTT);
    [LCS.TalusTT, ~] = LocalCoordinates(Coordinate.TalusTT);

    %% ---- Optional: Subtalar joint ----
    if jointSelection > 1
        % Load Calcaneus
        Bone_model.Calcaneus = stlread(fullfile(modelsFolder, 'AUT_Calcaneus.stl'));
        Bone_model_Auto.Calcaneus.Points = [Bone_model.Calcaneus.Points, ones(size(Bone_model.Calcaneus.Points,1),1)];

        % Talus & Calcaneus ST coordinate systems
        Coordinate.TalusST = load(fullfile(modelsFolder, 'Talus_ST_CS.txt'));
        Coordinate.CalcaneusST = load(fullfile(modelsFolder, 'Calcaneus_ST_CS.txt'));

        % Transformations (ST)
        [LCS.TalusST, ~] = LocalCoordinates(Coordinate.TalusST);
        [LCS.CalcaneusST, ~] = LocalCoordinates(Coordinate.CalcaneusST);
    end

    %% ---- Optional: Midfoot joints ----
    if jointSelection >= 3
        % Load Navicular
        Bone_model.Navicular = stlread(fullfile(modelsFolder, 'AUT_Navicular.stl'));
        Bone_model_Auto.Navicular.Points = [Bone_model.Navicular.Points, ones(size(Bone_model.Navicular.Points,1),1)];

        % Talus & Navicular TN coordinate systems
        Coordinate.TalusTN     = load(fullfile(modelsFolder, 'Talus_TN_CS.txt'));
        Coordinate.NavicularTN = load(fullfile(modelsFolder, 'Navicular_TN_CS.txt'));
        % Transformations (TN)
        [LCS.TalusTN,     ~] = LocalCoordinates(Coordinate.TalusTN);
        [LCS.NavicularTN, ~] = LocalCoordinates(Coordinate.NavicularTN);
    end
        if jointSelection >= 4
            Bone_model.Cuboid    = stlread(fullfile(modelsFolder, 'AUT_Cuboid.stl'));
            Bone_model_Auto.Cuboid.Points    = [Bone_model.Cuboid.Points, ones(size(Bone_model.Cuboid.Points,1),1)];
            % Calcaneus & Cuboid CC coordinate systems
            Coordinate.CalcaneusCC = load(fullfile(modelsFolder, 'Calcaneus_CC_CS.txt'));
            Coordinate.CuboidCC    = load(fullfile(modelsFolder, 'Cuboid_CC_CS.txt'));
    
            % Transformations (CC)
            [LCS.CalcaneusCC, ~] = LocalCoordinates(Coordinate.CalcaneusCC);
            [LCS.CuboidCC,    ~] = LocalCoordinates(Coordinate.CuboidCC);
        end
if jointSelection >= 5
            Bone_model.Fibula    = stlread(fullfile(modelsFolder, 'AUT_Fibula.stl'));
            Bone_model_Auto.Fibula.Points    = [Bone_model.Fibula.Points, ones(size(Bone_model.Fibula.Points,1),1)];
            % Fibula and  Tibia TF coordinate systems
            Coordinate.FibulaTF = load(fullfile(modelsFolder, 'Fibula_TF_CS.txt'));
            Coordinate.TibiaTF    = load(fullfile(modelsFolder, 'Tibia_TF_CS.txt'));
    
            % Transformations (CC)
            [LCS.FibulaTF, ~] = LocalCoordinates(Coordinate.FibulaTF);
            [LCS.TibiaTF,    ~] = LocalCoordinates(Coordinate.TibiaTF);
        end
    disp('3D bone models and respective local coordinate systems have been successfully loaded');
end

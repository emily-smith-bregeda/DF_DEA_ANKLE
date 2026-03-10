%% D. Transform bones to local coordinate system (LCS) (Transform2LCS.m)

function [Bone_model_LCS, Auto2LCS] = Transform2LCS(Bone_model, LCS, jointSelection)
% Transform2LCS - Transforms bone models and coordinate systems into Local Coordinate System (LCS)

% Initialize outputs
    Bone_model_LCS = struct();
    Auto2LCS = struct();
    
    % take the inverse of the transformation matrix made in BuildBoneModels
    % to apply later
    Auto2LCS.TibiaTT = inv(LCS.TibiaTT);
    Auto2LCS.TalusTT = inv(LCS.TalusTT);

    % pad the point cloud matrix with ones so we can multiply
    % now apply the tranformation matrix developed in BuildBoneModels by
    % multiplying the tranformation matrix to the bone model point cloud
    % do this for tibia and talus always
    
    Bone_model_LCS.TibiaTT.Points = (Auto2LCS.TibiaTT * [Bone_model.Tibia.Points, ones(size(Bone_model.Tibia.Points, 1), 1)]')';
    Bone_model_LCS.TalusTT.Points = (Auto2LCS.TalusTT * [Bone_model.Talus.Points, ones(size(Bone_model.Talus.Points, 1), 1)]')';
    
    % LCS_LCS.TibiaTT.Origin=(Auto2LCS.TibiaTT*[LCS_Auto.TibiaTT.Origin,1]')';
    % LCS_LCS.TibiaTT.X=(Auto2LCS.TibiaTT*[LCS_Auto.TibiaTT.X,1]')';
    % LCS_LCS.TibiaTT.Y=(Auto2LCS.TibiaTT*[LCS_Auto.TibiaTT.Y,1]')';
    % LCS_LCS.TibiaTT.Z=(Auto2LCS.TibiaTT*[LCS_Auto.TibiaTT.Z,1]')';
    % LCS_LCS.TalusTT.Origin=(Auto2LCS.TalusTT*[LCS_Auto.TalusTT.Origin,1]')';
    % LCS_LCS.TalusTT.X=(Auto2LCS.TalusTT*[LCS_Auto.TalusTT.X,1]')';
    % LCS_LCS.TalusTT.Y=(Auto2LCS.TalusTT*[LCS_Auto.TalusTT.Y,1]')';
    % LCS_LCS.TalusTT.Z=(Auto2LCS.TalusTT*[LCS_Auto.TalusTT.Z,1]')';
    
  % repeat for other joints if needed
    if jointSelection > 1
        % Transform Calcaneus for subtalar joint
        Auto2LCS.CalcaneusST = inv(LCS.CalcaneusST);
        Bone_model_LCS.CalcaneusST.Points = (Auto2LCS.CalcaneusST * [Bone_model.Calcaneus.Points, ones(size(Bone_model.Calcaneus.Points, 1), 1)]')';
        
        % Transform Talus for subtalar joint
        Auto2LCS.TalusST = inv(LCS.TalusST);
        Bone_model_LCS.TalusST.Points = (Auto2LCS.TalusST * [Bone_model.Talus.Points, ones(size(Bone_model.Talus.Points, 1), 1)]')';

    if jointSelection >= 3
        % Transform Navicular and Talus for TN joint
        Auto2LCS.NavicularTN = inv(LCS.NavicularTN);
        Bone_model_LCS.NavicularTN.Points = (Auto2LCS.NavicularTN * [Bone_model.Navicular.Points, ones(size(Bone_model.Navicular.Points, 1), 1)]')';
        Auto2LCS.TalusTN = inv(LCS.TalusTN);
        Bone_model_LCS.TalusTN.Points = (Auto2LCS.TalusTN * [Bone_model.Talus.Points, ones(size(Bone_model.Talus.Points, 1), 1)]')';
    end

    if jointSelection >= 4 
        % Transform Cuboid and Calcaneus for CC joint
        Auto2LCS.CuboidCC = inv(LCS.CuboidCC);
        Bone_model_LCS.CuboidCC.Points = (Auto2LCS.CuboidCC * [Bone_model.Cuboid.Points, ones(size(Bone_model.Cuboid.Points, 1), 1)]')';
        
        Auto2LCS.CalcaneusCC = inv(LCS.CalcaneusCC);
        Bone_model_LCS.CalcaneusCC.Points = (Auto2LCS.CalcaneusCC * [Bone_model.Calcaneus.Points, ones(size(Bone_model.Calcaneus.Points, 1), 1)]')';
    end

    disp('The bone models have been transformed from the autoscoper space to their local coordinate systems');
    end
   

end
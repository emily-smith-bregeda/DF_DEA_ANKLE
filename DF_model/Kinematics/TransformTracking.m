%% G. Transform dynamic and neutral tracking to LCS (TransformTracking.m)

function [Neutral_matx_LCS, Tracking_LCS, Angle_LCS, Translation_LCS] = TransformTracking(Neutral_Auto, Tracking_Auto, Auto2LCS, LCS, Lengths, prefix, jointSelection, side)

% TransformTibiotalar transforms Tibia and Talus tracking data from Auto to
% LCS
    % Initialize output structures
    Neutral_matx_LCS.(prefix) = struct();
    Tracking_LCS.(prefix) = struct();
    Angle_LCS.(prefix) = struct();
    Translation_LCS.(prefix) = struct();

    %% TibiaTT Transformation
    % Neutral transformation
    Neutral_matx_LCS.(prefix).TibiaTT = Neutral_Auto.(prefix).Tibia * LCS.TibiaTT;

    % Dynamic transformation
    for i = 1:Lengths.Tibia
        % Transform from Autoscoper to LCS
        Tracking_LCS.(prefix).TibiaTT(:,:,i) = Tracking_Auto.(prefix).Tibia(:,:,i) * LCS.TibiaTT;

        % Mirror left side
        if strcmpi(side, 'L')
            Tracking_LCS.(prefix).TibiaTT(1:3,1,i) = -Tracking_LCS.(prefix).TibiaTT(1:3,1,i);
            Tracking_LCS.(prefix).TibiaTT(1,4,i) = -Tracking_LCS.(prefix).TibiaTT(1,4,i);
        end
    end

    
    %% TalusTT Transformation
    % Neutral transformation
    Neutral_matx_LCS.(prefix).TalusTT = Neutral_Auto.(prefix).Talus * LCS.TalusTT;

    % Dynamic transformation
    for i = 1:Lengths.Talus
            Tracking_LCS.(prefix).TalusTT(:,:,i) = Tracking_Auto.(prefix).Talus(:,:,i) * LCS.TalusTT;

            % Mirror left side
            if strcmpi(side, 'L')
                Tracking_LCS.(prefix).TalusTT(1:3,1,i) = -Tracking_LCS.(prefix).TalusTT(1:3,1,i);
                Tracking_LCS.(prefix).TalusTT(1,4,i) = -Tracking_LCS.(prefix).TalusTT(1,4,i);
            end
    end

    disp('The neutral and dynamic tracking files have been transformed for the TIBIOTALAR joint');

    % Transform additional bones if applicable
     if jointSelection > 1
        % CalcaneusST Transformation (Subtalar Joint)
            Neutral_matx_LCS.(prefix).CalcaneusST = Neutral_Auto.(prefix).Calcaneus * LCS.CalcaneusST;
            Tracking_LCS.(prefix).CalcaneusST= NaN(4,4,Lengths.Calcaneus);
    
            for i = 1:Lengths.Calcaneus
                Tracking_LCS.(prefix).CalcaneusST(:,:,i) = Tracking_Auto.(prefix).Calcaneus(:,:,i) * LCS.CalcaneusST;
    
                % Flip medial-lateral direction for left foot
                if strcmp(side, 'L')
                    Tracking_LCS.(prefix).CalcaneusST(1:3,1,i) = -Tracking_LCS.(prefix).CalcaneusST(1:3,1,i); % Mirror Y-Z (change handedness)
                    Tracking_LCS.(prefix).CalcaneusST(1,1,i) = -Tracking_LCS.(prefix).CalcaneusST(1,4,i); % Flip X component of translation
                end
            end

            Neutral_matx_LCS.(prefix).TalusST = Neutral_Auto.(prefix).Talus * LCS.TalusST;
            Tracking_LCS.(prefix).TalusST = NaN(4,4,Lengths.Talus);
            for i = 1:Lengths.Talus
                Tracking_LCS.(prefix).TalusST(:,:,i) = Tracking_Auto.(prefix).Talus(:,:,i) * LCS.TalusST;
    
                % Flip medial-lateral direction for left foot
                if strcmp(side, 'L')
                Tracking_LCS.(prefix).TalusST(1:3,1,i) = -Tracking_LCS.(prefix).TalusST(1:3,1,i); % Mirror Y-Z (change handedness)
                Tracking_LCS.(prefix).TalusST(1,1,i) = -Tracking_LCS.(prefix).TalusST(1,4,i); % Flip X component of translation
                end
            end
            disp('The neutral and dynamic tracking files have been transformed for the SUBTALAR joint');    

        % All Joints
        if jointSelection >= 3
            % TalusTN and NavicularTN Transformation (Talonavicular joint)
            Neutral_matx_LCS.(prefix).TalusTN = Neutral_Auto.(prefix).Talus * LCS.TalusTN;
            Tracking_LCS.(prefix).TalusTN = NaN(4,4,Lengths.Talus);
    
            for i = 1:Lengths.Talus
                Tracking_LCS.(prefix).TalusTN(:,:,i) = Tracking_Auto.(prefix).Talus(:,:,i) * LCS.TalusTN;
    
                % Flip medial-lateral direction for left foot
                if strcmp(side, 'L')
                    Tracking_LCS.(prefix).TalusTN(1:3,1,i) = -Tracking_LCS.(prefix).TalusTN(1:3,1,i); % Mirror Y-Z (change handedness)
                    Tracking_LCS.(prefix).TalusTN(1,1,i) = -Tracking_LCS.(prefix).TalusTN(1,4,i); % Flip X component of translation
                end
               
            end
    
            Neutral_matx_LCS.(prefix).NavicularTN = Neutral_Auto.(prefix).Navicular * LCS.NavicularTN;
            Tracking_LCS.(prefix).NavicularTN = NaN(4,4,Lengths.Navicular);
            
                for i = 1:Lengths.Navicular
                    Tracking_LCS.(prefix).NavicularTN(:,:,i) = Tracking_Auto.(prefix).Navicular(:,:,i) * LCS.NavicularTN;
    
                    % Flip medial-lateral direction for left foot
                    if strcmp(side, 'L')
                        Tracking_LCS.(prefix).NavicularTN(1:3,1,i) = -Tracking_LCS.(prefix).NavicularTN(1:3,1,i); % Mirror Y-Z (change handedness)
                        Tracking_LCS.(prefix).NavicularTN(1,1,i) = -Tracking_LCS.(prefix).NavicularTN(1,4,i); % Flip X component of translation
                    end
    
                end
                disp('The neutral and dynamic tracking files have been transformed for the TALONAVICULAR joint');
        end
        % CalcaneusCC and CuboidCC Transformation (Calcaneocuboid Joint)
        if jointSelection >= 4
            Neutral_matx_LCS.(prefix).CalcaneusCC = Neutral_Auto.(prefix).Calcaneus * LCS.CalcaneusCC;
            Tracking_LCS.(prefix).CalcaneusCC = NaN(4,4,Lengths.Calcaneus);
    
            for i = 1:Lengths.Calcaneus
                Tracking_LCS.(prefix).CalcaneusCC(:,:,i) = Tracking_Auto.(prefix).Calcaneus(:,:,i) * LCS.CalcaneusCC;
    
                % Flip medial-lateral direction for left foot
                if strcmp(side, 'L')
                    Tracking_LCS.(prefix).CalcaneusCC(1:3,1,i) = -Tracking_LCS.(prefix).CalcaneusCC(1:3,1,i); % Mirror Y-Z (change handedness)
                    Tracking_LCS.(prefix).CalcaneusCC(1,1,i) = -Tracking_LCS.(prefix).CalcaneusCC(1,4,i); % Flip X component of translation
                end
            end
    
            Neutral_matx_LCS.(prefix).CuboidCC = Neutral_Auto.(prefix).Cuboid * LCS.CuboidCC;
            Tracking_LCS.(prefix).CuboidCC = NaN(4,4,Lengths.Cuboid);
            for i = 1:Lengths.Cuboid
                Tracking_LCS.(prefix).CuboidCC(:,:,i) = Tracking_Auto.(prefix).Cuboid(:,:,i) * LCS.CuboidCC;
    
                % Flip medial-lateral direction for left foot
                if strcmp(side, 'L')
                    Tracking_LCS.(prefix).CuboidCC(1:3,1,i) = -Tracking_LCS.(prefix).CuboidCC(1:3,1,i); % Mirror Y-Z (change handedness)
                    Tracking_LCS.(prefix).CuboidCC(1,1,i) = -Tracking_LCS.(prefix).CuboidCC(1,4,i); % Flip X component of translation
                end
            end
            disp('The neutral and dynamic tracking files have been transformed for the CALCANEOCUBOID joint');
        end
        % TibiaTF and FibulaTF Transformation (Calcaneocuboid Joint)
                if jointSelection >= 5
            Neutral_matx_LCS.(prefix).TibiaTF = Neutral_Auto.(prefix).Tibia * LCS.TibiaTF;
            Tracking_LCS.(prefix).TibiaTF = NaN(4,4,Lengths.Tibia);
    
            for i = 1:Lengths.Tibia
                Tracking_LCS.(prefix).TibaiTF(:,:,i) = Tracking_Auto.(prefix)Tibia(:,:,i) * LCS.TibiaTF;
    
                % Flip medial-lateral direction for left foot
                if strcmp(side, 'L')
                    Tracking_LCS.(prefix).CalcaneusCC(1:3,1,i) = -Tracking_LCS.(prefix).CalcaneusCC(1:3,1,i); % Mirror Y-Z (change handedness)
                    Tracking_LCS.(prefix).CalcaneusCC(1,1,i) = -Tracking_LCS.(prefix).CalcaneusCC(1,4,i); % Flip X component of translation
                end
            end
    
            Neutral_matx_LCS.(prefix).CuboidCC = Neutral_Auto.(prefix).Cuboid * LCS.CuboidCC;
            Tracking_LCS.(prefix).CuboidCC = NaN(4,4,Lengths.Cuboid);
            for i = 1:Lengths.Cuboid
                Tracking_LCS.(prefix).CuboidCC(:,:,i) = Tracking_Auto.(prefix).Cuboid(:,:,i) * LCS.CuboidCC;
    
                % Flip medial-lateral direction for left foot
                if strcmp(side, 'L')
                    Tracking_LCS.(prefix).CuboidCC(1:3,1,i) = -Tracking_LCS.(prefix).CuboidCC(1:3,1,i); % Mirror Y-Z (change handedness)
                    Tracking_LCS.(prefix).CuboidCC(1,1,i) = -Tracking_LCS.(prefix).CuboidCC(1,4,i); % Flip X component of translation
                end
            end
            disp('The neutral and dynamic tracking files have been transformed for the CALCANEOCUBOID joint');
        end
     end

end



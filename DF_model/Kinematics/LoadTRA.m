%% F. Loading Dynamic and Neutral Tracking Data (LoadTRA.m)

function [Tracking_Auto, Neutral_Auto, Tracking_Data, trackingFolder, Lengths, Starts] = ...
    LoadTRA(trialNumStr, side, folderName, folderPath, prefix, jointSelection)

    Tracking_Auto.(prefix) = struct();
    Neutral_Auto.(prefix)  = struct();
    Tracking_Data = struct();
    Lengths = struct();
    Starts  = struct();

    trackingFolder = fullfile(folderPath, 'Tracking');

    %%
    % Helper function to load dynamic tracking for one bone
    function loadDynamicBone(boneName)
        Tracking_file = fullfile(trackingFolder, ...
            [folderName '_' boneName '.tra']);

        Tracking_Data.(boneName) = importdata(Tracking_file);
        numFrames = size(Tracking_Data.(boneName), 1);

        Tracking_Auto.(prefix).(boneName) = nan(4, 4, numFrames);

        for i = 1:numFrames
            Tracking_Auto.(prefix).(boneName)(:, :, i) = ...
                reshape(Tracking_Data.(boneName)(i, :), 4, 4)';
        end

        % Interpolate NaNs
        for r = 1:4
            for c = 1:4
                data = squeeze(Tracking_Auto.(prefix).(boneName)(r, c, :));
                nanIdx = isnan(data);
                if any(nanIdx) && ~all(nanIdx)
                    validIdx = find(~nanIdx);
                    Tracking_Auto.(prefix).(boneName)(r, c, nanIdx) = ...
                        interp1(validIdx, data(validIdx), find(nanIdx), ...
                                'linear', 'extrap');
                end
            end
        end

        Lengths.(boneName) = numFrames;
        Starts.(boneName)  = ...
            ceil(find(~isnan(Tracking_Auto.(prefix).(boneName)), 1) / 16);
    end

    %% 
    % Helper function to load neutral tracking for one bone
    function loadNeutralBone(boneName)
        Neutral_file = fullfile(neutralFolder, ...
            [trialNumStr '_Neutral_' side '_' boneName '.tra']);

        Tracking_Data.([boneName '_Neutral']) = importdata(Neutral_file);
        Neutral_Auto.(prefix).(boneName) = ...
            reshape(Tracking_Data.([boneName '_Neutral'])(1, :), 4, 4)';
    end

    %%
    % Load dynamic tracking 
    loadDynamicBone('Tibia');
    loadDynamicBone('Talus');

    if jointSelection >= 2
        loadDynamicBone('Calcaneus');
    end

    if jointSelection >= 3
        loadDynamicBone('Navicular');
    end

    if jointSelection >= 4
        loadDynamicBone('Cuboid');
    end
    if jointSelection >= 5
        loadDynamicBone('Fibula');
    end
    %%
    % Load neutral tracking
    parentFolder  = fileparts(folderPath);
    neutralFolder = fullfile(parentFolder, ...
        [trialNumStr '_Neutral_' side], 'Tracking');

    loadNeutralBone('Tibia');
    loadNeutralBone('Talus');

    if jointSelection >= 2
        loadNeutralBone('Calcaneus');
    end

    if jointSelection >= 3
        loadNeutralBone('Navicular');
    end

    if jointSelection >= 4
        loadNeutralBone('Cuboid');
    end
    if jointSelection >= 5
        loadNeutralBone('Fibula');
    end
    disp('The dynamic and neutral tracking files have been successfully loaded');

end

%% J. Interpolating the data (InterpJointData.m)
% interpolates data points to be within 100% stance phase i.e. normalizes to 101 points

function [Norm_RelAngle_interp] = InterpJointData(Filtered_RelAngle_LCS_Norm, prefix, heelStrikeFrame, toeOffFrame, jointSelection)

% Initialize output
Norm_RelAngle_interp.(prefix) = struct();

%% Always interpolate Tibiotalar
dataLength = size(Filtered_RelAngle_LCS_Norm.(prefix).Tibiotalar, 1);
validToeOffFrame     = min(toeOffFrame, dataLength);
validHeelStrikeFrame = min(heelStrikeFrame, dataLength);

validFrameIndices = linspace(validHeelStrikeFrame, validToeOffFrame, validToeOffFrame - validHeelStrikeFrame + 1)';
normFrames = linspace(validHeelStrikeFrame, validToeOffFrame, 101);

jointAngles_Norm = Filtered_RelAngle_LCS_Norm.(prefix).Tibiotalar(validHeelStrikeFrame:validToeOffFrame, :, :);
Norm_RelAngle_interp.(prefix).Tibiotalar = interp1(validFrameIndices, jointAngles_Norm, normFrames, 'linear', NaN);

%% Subtalar
if jointSelection > 1
    dataLength = size(Filtered_RelAngle_LCS_Norm.(prefix).Subtalar, 1);
    validToeOffFrame     = min(toeOffFrame, dataLength);
    validHeelStrikeFrame = min(heelStrikeFrame, dataLength);

    jointAngles_Norm = Filtered_RelAngle_LCS_Norm.(prefix).Subtalar(validHeelStrikeFrame:validToeOffFrame, :, :);
    Norm_RelAngle_interp.(prefix).Subtalar = interp1(validFrameIndices, jointAngles_Norm, normFrames, 'linear', NaN);
end

%% Talonavicular
if jointSelection >= 3
    dataLength = size(Filtered_RelAngle_LCS_Norm.(prefix).Talonavicular, 1);
    validToeOffFrame     = min(toeOffFrame, dataLength);
    validHeelStrikeFrame = min(heelStrikeFrame, dataLength);

    jointAngles_Norm = Filtered_RelAngle_LCS_Norm.(prefix).Talonavicular(validHeelStrikeFrame:validToeOffFrame, :, :);
    Norm_RelAngle_interp.(prefix).Talonavicular = interp1(validFrameIndices, jointAngles_Norm, normFrames, 'linear', NaN);
end

%% Calcaneocuboid
if jointSelection >= 4
    dataLength = size(Filtered_RelAngle_LCS_Norm.(prefix).Calcaneocuboid, 1);
    validToeOffFrame     = min(toeOffFrame, dataLength);
    validHeelStrikeFrame = min(heelStrikeFrame, dataLength);

    jointAngles_Norm = Filtered_RelAngle_LCS_Norm.(prefix).Calcaneocuboid(validHeelStrikeFrame:validToeOffFrame, :, :);
    Norm_RelAngle_interp.(prefix).Calcaneocuboid = interp1(validFrameIndices, jointAngles_Norm, normFrames, 'linear', NaN);
end

disp('Interpolation complete');

end

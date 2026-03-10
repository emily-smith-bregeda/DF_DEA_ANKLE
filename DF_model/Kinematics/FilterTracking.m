%% H. Filter LCS Tracking Matrices for Visualization (FilterTracking.m)

function Filtered_Tracking_LCS = FilterTracking(Tracking_LCS, prefix, cutoff_freq, sampling_freq, jointSelection)
% FilterTracking applies a 4th-order Butterworth low-pass filter
% to the TibiaTT and TalusTT tracking data in Tracking_LCS.

    % Calculate the normalized cutoff frequency
    Wn = cutoff_freq / (sampling_freq / 2);
    
    % Design a 4th-order Butterworth low-pass filter
    [b, a] = butter(4, Wn, 'low');
    
    % Initialize output structure
    Filtered_Tracking_LCS.(prefix).TibiaTT = NaN(size(Tracking_LCS.(prefix).TibiaTT));
    Filtered_Tracking_LCS.(prefix).TalusTT = NaN(size(Tracking_LCS.(prefix).TalusTT));
    if jointSelection > 1
        Filtered_Tracking_LCS.(prefix).TalusST = NaN(size(Tracking_LCS.(prefix).TalusST));
        Filtered_Tracking_LCS.(prefix).CalcaneusST = NaN(size(Tracking_LCS.(prefix).CalcaneusST));
        if jointSelection >= 3
            Filtered_Tracking_LCS.(prefix).TalusTN = NaN(size(Tracking_LCS.(prefix).TalusTN));
            Filtered_Tracking_LCS.(prefix).NavicularTN = NaN(size(Tracking_LCS.(prefix).NavicularTN));
        end
        if jointSelection >= 4
            Filtered_Tracking_LCS.(prefix).CalcaneusCC = NaN(size(Tracking_LCS.(prefix).CalcaneusCC));
            Filtered_Tracking_LCS.(prefix).CuboidCC = NaN(size(Tracking_LCS.(prefix).CuboidCC));
        end
    end
    
    % Filter function for a single 4x4xN matrix
    function filtered_matrix = filterMatrix(data_matrix)
        filtered_matrix = NaN(size(data_matrix));
        for row = 1:4
            for col = 1:4
                time_series_data = squeeze(data_matrix(row, col, :));
                valid_indices = find(~isnan(time_series_data));
                if ~isempty(valid_indices)
                    valid_data = time_series_data(valid_indices);
                    filtered_data = filtfilt(b, a, valid_data);
                    filtered_matrix(row, col, valid_indices) = filtered_data;
                end
            end
        end
    end

    % Apply filtering
    Filtered_Tracking_LCS.(prefix).TibiaTT = filterMatrix(Tracking_LCS.(prefix).TibiaTT);
    Filtered_Tracking_LCS.(prefix).TalusTT = filterMatrix(Tracking_LCS.(prefix).TalusTT);
    if jointSelection > 1
        Filtered_Tracking_LCS.(prefix).TalusST = filterMatrix(Tracking_LCS.(prefix).TalusST);
        Filtered_Tracking_LCS.(prefix).CalcaneusST = filterMatrix(Tracking_LCS.(prefix).CalcaneusST);
        if jointSelection >= 3
            Filtered_Tracking_LCS.(prefix).TalusTN = filterMatrix(Tracking_LCS.(prefix).TalusTN);
            Filtered_Tracking_LCS.(prefix).NavicularTN = filterMatrix(Tracking_LCS.(prefix).NavicularTN);
        end
        if jointSelection >= 4
            Filtered_Tracking_LCS.(prefix).CalcaneusCC = filterMatrix(Tracking_LCS.(prefix).CalcaneusCC);
            Filtered_Tracking_LCS.(prefix).CuboidCC = filterMatrix(Tracking_LCS.(prefix).CuboidCC);
        end
    end

    disp('The dynamic tracking files have been filtered');
end

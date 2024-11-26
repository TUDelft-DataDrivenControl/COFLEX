function tablesPath = create_LUT_for_WSE(inputFile, outputFolder, outputFile)
    % This function creates a 3D LUT from a four-column text file (PITCH | WNDSPD | ROTSPD | CP)
    %
    % Arguments:
    %   inputFile   - Path to the input text file
    %   outputFolder - Path to the output folder where the .mat file will be saved
    %   outputFile   - Name of the output .mat file (e.g., 'Tables.mat')
    %
    % Returns:
    %   tablesPath - Full path to the saved .mat file

    % Load the data from the specified text file
    data = dlmread(inputFile, '\t', 1, 0);

    % Extract unique values for Pitch, Wind Speed, and Rotational Speed
    Pitch_unique = unique(data(:, 1));  % Pitch is in the first column
    wndSpd_unique = unique(data(:, 2)); % Wind Speed is in the second column
    rotSpd_unique = unique(data(:, 3)); % Rotational Speed is in the third column

    % Initialize Cp matrix
    Cp = zeros(length(Pitch_unique), length(wndSpd_unique), length(rotSpd_unique));

    % Initialize progress bar
    f = waitbar(0, 'Starting');
    n = length(Pitch_unique) * length(wndSpd_unique) * length(rotSpd_unique);
    idx = 1;

    % Fill Cp matrix with corresponding Cp values
    for i = 1:length(Pitch_unique)
        for j = 1:length(wndSpd_unique)
            for k = 1:length(rotSpd_unique)
                Cp(i, j, k) = data(idx, 4);  % Cp is in the fourth column
                idx = idx + 1;
                waitbar(idx / n, f, sprintf('Progress: %d %%', floor(idx / n * 100)));
            end
        end
    end
    close(f);

    % Create a struct named Tables
    Tables.Pitch = Pitch_unique;
    Tables.wndSpd = wndSpd_unique;
    Tables.rotSpd = rotSpd_unique;
    Tables.Cp = Cp;

    % Save the struct to a .mat file in the specified output location
    tablesPath = fullfile(outputFolder, outputFile);
    save(tablesPath, 'Tables');
end
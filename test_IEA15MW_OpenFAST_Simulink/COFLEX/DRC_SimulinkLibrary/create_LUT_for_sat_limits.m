function satLimPath = create_LUT_for_sat_limits(inputFile, outputFolder, outputFile)
    % This function reads data from a text file, extracts relevant columns, and saves
    % them in a .mat file as a structured variable.
    %
    % Arguments:
    %   inputFile   - Path to the input text file
    %   outputFolder - Path to the output folder where the .mat file will be saved
    %   outputFile   - Name of the output .mat file (e.g., 'sat_lim_constrtip18.mat')
    %
    % Returns:
    %   satLimPath - Full path to the saved .mat file

    % Read the file
    fileID = fopen(inputFile, 'r');
    if fileID == -1
        error('Could not open the input file.');
    end

    % Define the number of columns and format string
    N_col = 12;
    formatString = repmat('%f', 1, N_col);

    % Read data into a cell array, skipping the header line
    data = textscan(fileID, formatString, 'HeaderLines', 1, 'Delimiter', ',');
    fclose(fileID);

    % Extract the columns of interest and perform necessary conversions
    ws_sat_lim     = data{1};       % Windspeed (m/s)
    torque_sat_lim = data{5} * 1000; % Gen. Torque (kNm) converted to Nm
    omega_sat_lim  = data{6};       % Rotor Rotational Speed (rad/s)
    pitch_sat_lim  = data{7};       % Pitch (rad)

    % Save the variables in a structure
    sat_lim.ws_sat_lim     = ws_sat_lim;
    sat_lim.torque_sat_lim = torque_sat_lim;
    sat_lim.omega_sat_lim  = omega_sat_lim;
    sat_lim.pitch_sat_lim  = pitch_sat_lim;

    % Construct the full path for the output file
    satLimPath = fullfile(outputFolder, outputFile);

    % Save the structure to the specified .mat file
    save(satLimPath, 'sat_lim');

    % Display basic statistics for the loaded vectors
    fprintf('Wind Speed (m/s) - Min: %.2f, Max: %.2f, Elements: %d, Shape: [%d, 1]\n', ...
        min(ws_sat_lim), max(ws_sat_lim), numel(ws_sat_lim), size(ws_sat_lim, 1));
    fprintf('Pitch (rad) - Min: %.2f, Max: %.2f, Elements: %d, Shape: [%d, 1]\n', ...
        min(pitch_sat_lim), max(pitch_sat_lim), numel(pitch_sat_lim), size(pitch_sat_lim, 1));

end
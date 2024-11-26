function lutPath = create_LUT_for_FF(inputFile, outputFolder, outputFile)
    % This function reads data from a text file, extracts relevant columns, and saves
    % them in a .mat file as a structured variable.
    %
    % Arguments:
    %   inputFile   - Path to the input text file
    %   outputFolder - Path to the output folder where the .mat file will be saved
    %   outputFile   - Name of the output .mat file (e.g., 'LUT_FF_constrtip10.mat')
    %
    % Returns:
    %   lutPath - Full path to the saved .mat file

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
    ws_FF     = data{1};       % Windspeed (m/s)
    torque_FF = data{5} * 1000; % Gen. Torque (kNm) converted to Nm
    omega_FF  = data{6};       % Rotor Rotational Speed (rad/s)
    pitch_FF  = data{7};       % Pitch (rad)

    % Save the variables in a structure
    LUT_FF.ws_FF     = ws_FF;
    LUT_FF.torque_FF = torque_FF;
    LUT_FF.omega_FF  = omega_FF;
    LUT_FF.pitch_FF  = pitch_FF;

    % Construct the full path for the output file
    lutPath = fullfile(outputFolder, outputFile);

    % Save the structure to the specified .mat file
    save(lutPath, 'LUT_FF');


    % Display basic statistics for the loaded vectors
    fprintf('Wind Speed (m/s) - Min: %.2f, Max: %.2f, Elements: %d, Shape: [%d, 1]\n', ...
        min(ws_FF), max(ws_FF), numel(ws_FF), size(ws_FF, 1));
    fprintf('Pitch (rad) - Min: %.2f, Max: %.2f, Elements: %d, Shape: [%d, 1]\n', ...
        min(pitch_FF), max(pitch_FF), numel(pitch_FF), size(pitch_FF, 1));
    fprintf('Omega (rad/s) - Min: %.2f, Max: %.2f, Elements: %d, Shape: [%d, 1]\n', ...
            min(omega_FF), max(omega_FF), numel(omega_FF), size(omega_FF, 1));
    fprintf('Torque (Nm) - Min: %.2f, Max: %.2f, Elements: %d, Shape: [%d, 1]\n', ...
        min(torque_FF), max(torque_FF), numel(torque_FF), size(torque_FF, 1));

end
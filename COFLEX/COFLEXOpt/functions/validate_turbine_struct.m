function validate_turbine_struct(turbine, requiredFields)
    % Display all fields currently present in turbine struct
    turbineFields = fieldnames(turbine);
    fprintf('\nAvailable fields in turbine struct:\n');
    
    % Display each field name with a note if it is a symbolic function
    for i = 1:length(turbineFields)
        fieldName = turbineFields{i};
        fieldValue = turbine.(fieldName);
        
        if isa(fieldValue, 'sym') || isa(fieldValue, 'function_handle')
            fprintf('  %-15s (symbolic/function)\n', fieldName);
        else
            fprintf('  %-15s: %s\n', fieldName, mat2str(fieldValue));
        end
    end
    
    % Prepare to track missing fields by constraint group
    missingFields = struct();
    
    % Check for missing fields in each constraint group
    constraints = fieldnames(requiredFields);
    for k = 1:length(constraints)
        constraint = constraints{k};
        requiredFieldsForConstraint = requiredFields.(constraint);
        
        % Ensure requiredFieldsForConstraint is a cell array of strings
        if ~iscell(requiredFieldsForConstraint)
            requiredFieldsForConstraint = {requiredFieldsForConstraint}; % Convert to cell if needed
        end
        % Filter out non-string entries
        requiredFieldsForConstraint = requiredFieldsForConstraint(cellfun(@ischar, requiredFieldsForConstraint));
        
        missingFields.(constraint) = {};
        
        % Check each field in the current constraint group
        for i = 1:length(requiredFieldsForConstraint)
            fieldToCheck = requiredFieldsForConstraint{i};  % Extract the field name as a string
            if ~isfield(turbine, fieldToCheck)
                missingFields.(constraint){end+1} = fieldToCheck;
            end
        end
    end
    
    % Display and handle missing fields
    for k = 1:length(constraints)
        constraint = constraints{k};
        if ~isempty(missingFields.(constraint))
            % Construct error message
            errorMessage = sprintf('Error: The following required fields for %s are missing in the turbine struct: %s', ...
                constraint, strjoin(missingFields.(constraint), ', '));
            
            % Throw an error to stop execution
            error(errorMessage);
        else
            fprintf('%s validated successfully. All required fields are present.\n', constraint);
        end
    end
end

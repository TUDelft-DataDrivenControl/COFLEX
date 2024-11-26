% Automatically extract required fields from constraints
function requiredFields = extract_required_fields(g_constraints)
    % Initialize required fields as an empty struct
    requiredFields = struct();

    % Dynamically get field names from each constraints structure
    requiredFields.g_constraints = fieldnames(g_constraints);
end


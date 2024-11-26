function replaceinfile(INFILE, OUTFILE, SEARCHSTRING, REPLACESTRING, varargin)
% REPLACEINFILE - Replaces a specific string part in a text file
% replaceinfile(INFILE, OUTFILE, SEARCHSTRING, REPLACESTRING) - Searches
% for an exact match of SEARCHSTRING in the external file INFILE. When a
% match in found the line is replaced by REPLACESTRING. A new file is
% written to OUTFILE, which can either be the same as INFILE or defined as
% a new file.
% 
% replaceinfile(INFILE, OUTFILE, SEARCHSTRING, REPLACESTRING, DELIM, SEARCHLOC, REPLACELOC, EXACTMATCH)
% When providing a delimiter DELIM, you can specifify at which delimiter location 
% a match with SEARCHSTRING should be found. When a match is found,
% REPLACESTRING is written to delimiter location REPLACELOC.
%
% S.P. Mulders (Sebastiaan)
% Delft Center for Systems and Control (DCSC)
% The Netherlands, 2021


% Assertions
if ~isempty(varargin)
    if length(varargin) == 4
        DELIM = char(varargin{1});
        SEARCHLOC = varargin{2};
        REPLACELOC = varargin{3};
        EXACTMATCH = varargin{4};
    else
        error('When providing a delimiter, also searchLocation and replaceLocation should be provided');
    end
end

if ~isa(DELIM, 'char')
    error('Delimiter should be a string');
end
if ~isa(SEARCHLOC, 'numeric')
    error('searchLocation should be an integer');
end
if ~isa(REPLACELOC, 'numeric')
    error('replaceLocation should be an integer');
end

% If no OUTFILE is specified, overwrite INFILE
if isempty(OUTFILE)
    OUTFILE = INFILE;
end


% Some preprocessing
SEARCHSTRING = char(SEARCHSTRING);
REPLACESTRING = char(REPLACESTRING);

% Read the file
fid = fopen(INFILE);
data = textscan(fid, '%s', 'Delimiter', '\n', 'CollectOutput', true);
fclose(fid);
% modify the cell array
% find the position where changes need to be applied and insert new data
for ii = 1:length(data{1})
    if exist('DELIM', 'var')
        data_split = split(data{1}{ii}, DELIM);
        data_split = data_split(~cellfun('isempty',data_split));
        
        if length(data_split) < SEARCHLOC || length(data_split) < REPLACELOC
            continue
        end
        
        if EXACTMATCH
            stringFound = strcmp(data_split{SEARCHLOC}, SEARCHSTRING);
        else
            stringFound = contains(data_split{SEARCHLOC}, SEARCHSTRING);
        end
        
        if ~stringFound
            continue
        else
            data_split{REPLACELOC} = REPLACESTRING;
            data{1}{ii} = strjoin(data_split, DELIM);
        end
    else
        stringFound = strcmp(data{1}{ii}, SEARCHSTRING); % search for this string in the array
        if stringFound
            data{1}{ii} = REPLACESTRING; % replace with this string
        end
    end
end
    
% write the modified cell array into the text file
fid = fopen(OUTFILE, 'w');
for ii = 1:length(data{1})
    fprintf(fid, '%s\n', char(data{1}{ii}));
end
fclose(fid);
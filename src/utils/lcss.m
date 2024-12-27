function [longestSeq, cellIdx] = lcss(cellArray, inputArray)
% Initialize variables
maxLen = 0;
cellIdx = [];
longestSeq = [];

% Precompute the length of the input array
inputLen = numel(inputArray);

% Iterate through the cell array
for i = 1:numel(cellArray)
    % Get the current array
    currentArray = cellArray{i};
    currentLen = numel(currentArray);
    
    % Determine the maximum comparison length
    compareLen = min(inputLen, currentLen);
    
    % Compare arrays for the starting sequence
    matchMask = currentArray(1:compareLen) == inputArray(1:compareLen);
    matchLen = find(~matchMask, 1) - 1;
    
    % Handle the case where all elements match
    if isempty(matchLen)
        matchLen = compareLen;
    end
    
    % Update the longest sequence if necessary
    if matchLen > maxLen
        maxLen = matchLen;
        longestSeq = currentArray(1:matchLen);
        cellIdx = i;
    end
end

end
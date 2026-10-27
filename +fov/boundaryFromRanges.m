function boundary = boundaryFromRanges(observerPose, angles, ranges, openingAngle, isJump)
%BOUNDARYFROMRANGES Construct a sampled world boundary from relative ranges.
%
%   For a partial FOV the result is observer, endpoints, observer. For a
%   full circle it is endpoints followed by the first endpoint.
%
%   boundary = fov.boundaryFromRanges(observerPose, angles, ranges, ...
%       openingAngle, isJump) additionally flags consecutive ray pairs that
%   cross a depth discontinuity. isJump is a logical column of length N-1
%   for a partial FOV (pair i is rays i, i+1) or length N for a full 2*pi
%   FOV (the last entry is the wrap pair N, 1). At a flagged pair, a step
%   vertex is inserted at the nearer range along the farther ray's
%   direction, so the polygon goes radially out instead of drawing a
%   diagonal chord into the occluded shadow. Omitting isJump (or passing
%   it empty) reproduces the original unbridged construction exactly.

if ~(isnumeric(observerPose) && isreal(observerPose) && ...
        numel(observerPose) == 3 && all(isfinite(observerPose(:))))
    error('fov:boundaryFromRanges:InvalidPose', ...
        'observerPose must contain three finite real numeric values.');
end
if ~(isnumeric(angles) && isreal(angles) && iscolumn(angles) && ...
        numel(angles) >= 2 && all(isfinite(angles)))
    error('fov:boundaryFromRanges:InvalidAngles', ...
        'angles must be a finite column vector with at least two entries.');
end
if ~(isnumeric(ranges) && isreal(ranges) && iscolumn(ranges) && ...
        numel(ranges) == numel(angles) && all(isfinite(ranges)) && ...
        all(ranges >= 0))
    error('fov:boundaryFromRanges:InvalidRanges', ...
        'ranges must be a nonnegative finite column vector matching angles.');
end
if ~(isnumeric(openingAngle) && isscalar(openingAngle) && isreal(openingAngle) && ...
        isfinite(openingAngle) && openingAngle > 0 && openingAngle <= 2*pi)
    error('fov:boundaryFromRanges:InvalidOpeningAngle', ...
        'openingAngle must be in the interval (0, 2*pi].');
end

numRays = numel(angles);
isFullFov = double(openingAngle) == 2*pi;
numPairs = numRays - 1;
if isFullFov
    numPairs = numRays;
end
if nargin < 5 || isempty(isJump)
    isJump = false(0, 1);
elseif ~(islogical(isJump) && iscolumn(isJump) && numel(isJump) == numPairs)
    error('fov:boundaryFromRanges:InvalidIsJump', ...
        'isJump must be a logical column vector with %d entries, or empty.', numPairs);
end

% Convert relative ray angles into world endpoints at the current observer pose.
observerPose = reshape(double(observerPose), 1, 3);
worldAngles = observerPose(3) + double(angles);
ranges = double(ranges);
endPoints = bsxfun(@plus, observerPose(1:2), bsxfun(@times, ...
    ranges, [cos(worldAngles), sin(worldAngles)]));

if isempty(isJump)
    % A partial FOV closes through the observer; a full FOV closes around its rim.
    if isFullFov
        boundary = [endPoints; endPoints(1, :)];
    else
        boundary = [observerPose(1:2); endPoints; observerPose(1:2)];
    end
    return;
end

% A far ray closed off on both sides (by the aperture edge or a jump to a
% nearer ray) would trace a zero-area out-and-back spike along itself, which
% is self-intersecting. Pull it in to its farther near neighbour instead.
previousJump = [~isFullFov || isJump(end); isJump(1:numRays - 1)];
nextJump = [isJump(1:numRays - 1); ~isFullFov || isJump(end)];
previousRange = [-Inf; ranges(1:end - 1)];
nextRange = [ranges(2:end); -Inf];
if isFullFov
    previousRange(1) = ranges(end);
    nextRange(end) = ranges(1);
end
isSpike = previousJump & previousRange < ranges & nextJump & nextRange < ranges;
ranges(isSpike) = max(previousRange(isSpike), nextRange(isSpike));
endPoints = bsxfun(@plus, observerPose(1:2), bsxfun(@times, ...
    ranges, [cos(worldAngles), sin(worldAngles)]));

% Walk consecutive ray pairs, inserting a conservative step vertex at jumps
% instead of a diagonal chord into the shadow. A full FOV's last pair wraps.
vertices = endPoints(1, :);
for pairIndex = 1:numPairs
    firstIndex = pairIndex;
    secondIndex = mod(pairIndex, numRays) + 1;
    if isJump(pairIndex)
        firstRange = ranges(firstIndex);
        secondRange = ranges(secondIndex);
        if firstRange < secondRange
            vertices = [vertices; observerPose(1:2) + firstRange * ...
                [cos(worldAngles(secondIndex)), sin(worldAngles(secondIndex))]]; %#ok<AGROW>
        elseif secondRange < firstRange
            vertices = [vertices; observerPose(1:2) + secondRange * ...
                [cos(worldAngles(firstIndex)), sin(worldAngles(firstIndex))]]; %#ok<AGROW>
        end
    end
    vertices = [vertices; endPoints(secondIndex, :)]; %#ok<AGROW>
end

if isFullFov
    boundary = vertices;
else
    boundary = [observerPose(1:2); vertices; observerPose(1:2)];
end
end

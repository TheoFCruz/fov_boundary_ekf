function boundary = boundaryFromRanges(observerPose, angles, ranges, openingAngle)
%BOUNDARYFROMRANGES Construct a sampled world boundary from relative ranges.
%
%   For a partial FOV the result is observer, endpoints, observer. For a
%   full circle it is endpoints followed by the first endpoint.

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

% Convert relative ray angles into world endpoints at the current observer pose.
observerPose = reshape(double(observerPose), 1, 3);
worldAngles = observerPose(3) + double(angles);
endPoints = bsxfun(@plus, observerPose(1:2), bsxfun(@times, ...
    double(ranges), [cos(worldAngles), sin(worldAngles)]));
% A partial FOV closes through the observer; a full FOV closes around its rim.
if double(openingAngle) == 2*pi
    boundary = [endPoints; endPoints(1, :)];
else
    boundary = [observerPose(1:2); endPoints; observerPose(1:2)];
end
end

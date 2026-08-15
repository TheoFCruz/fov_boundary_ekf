function rayAngles = sampleRayAngles(observer, varargin)
%SAMPLERAYANGLES Sample uniformly spaced directions over an observer's FOV.
%
%   rayAngles = fov.sampleRayAngles(observer, numRays)
%   rayAngles = fov.sampleRayAngles(observer, 'NumRays', numRays)
%
% The returned angles are a column vector in ascending counterclockwise
% order. For a partial FOV, both boundary angles are included. For a full
% 2*pi FOV, the endpoint is omitted so that the first direction is not
% duplicated.

if ~isa(observer, 'fov.Observer')
    error('fov:sampleRayAngles:InvalidObserver', ...
        'observer must be an fov.Observer object.');
end

if nargin == 2 && isnumeric(varargin{1})
    numRays = varargin{1};
else
    parser = inputParser;
    parser.FunctionName = 'fov.sampleRayAngles';
    addParameter(parser, 'NumRays', []);
    parse(parser, varargin{:});
    numRays = parser.Results.NumRays;
end

if ~(isnumeric(numRays) && isscalar(numRays) && isreal(numRays) && ...
        isfinite(numRays) && numRays >= 2 && floor(numRays) == numRays)
    error('fov:sampleRayAngles:InvalidNumRays', ...
        'NumRays must be an integer scalar greater than or equal to 2.');
end

numRays = double(numRays);
startAngle = observer.Heading - observer.OpeningAngle/2;

if observer.OpeningAngle == 2*pi
    rayAngles = linspace(startAngle, startAngle + 2*pi, numRays + 1).';
    rayAngles(end) = [];
else
    rayAngles = linspace( ...
        startAngle, startAngle + observer.OpeningAngle, numRays).';
end
end

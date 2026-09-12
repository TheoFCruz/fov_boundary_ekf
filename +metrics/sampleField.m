function field = sampleField(evaluator, varargin)
%SAMPLEFIELD Evaluate a pointwise metric on a rectangular Cartesian grid.
%
%   field = metrics.sampleField(evaluator, ...
%       'Bounds', [xmin, xmax, ymin, ymax], 'GridSize', [ny, nx])
%
% evaluator must accept an N-by-2 point array and return an N-by-1 finite,
% real numeric vector. The returned field uses meshgrid orientation: rows
% increase with y and columns increase with x.

if ~isa(evaluator, 'function_handle')
    error('metrics:sampleField:InvalidEvaluator', ...
        'evaluator must be a function handle.');
end

parser = inputParser;
parser.FunctionName = 'metrics.sampleField';
addParameter(parser, 'Bounds', []);
addParameter(parser, 'GridSize', [201, 201]);
addParameter(parser, 'Name', 'Metric field');
addParameter(parser, 'Units', '');
parse(parser, varargin{:});
options = parser.Results;

bounds = options.Bounds;
gridSize = options.GridSize;

if ~(isnumeric(bounds) && isreal(bounds) && numel(bounds) == 4 && ...
        all(isfinite(bounds(:))))
    error('metrics:sampleField:InvalidBounds', ...
        'Bounds must be [xmin, xmax, ymin, ymax] with finite values.');
end
bounds = reshape(double(bounds), 1, 4);
if bounds(1) >= bounds(2) || bounds(3) >= bounds(4)
    error('metrics:sampleField:InvalidBounds', ...
        'Bounds must satisfy xmin < xmax and ymin < ymax.');
end

if ~(isnumeric(gridSize) && isreal(gridSize) && numel(gridSize) == 2 && ...
        all(isfinite(gridSize(:))) && all(gridSize(:) >= 2) && ...
        all(floor(gridSize(:)) == gridSize(:)))
    error('metrics:sampleField:InvalidGridSize', ...
        'GridSize must be a two-element integer vector with values at least 2.');
end
gridSize = reshape(double(gridSize), 1, 2);

if ~(ischar(options.Name) || (isstring(options.Name) && isscalar(options.Name)))
    error('metrics:sampleField:InvalidName', ...
        'Name must be a character vector or string scalar.');
end
if ~(ischar(options.Units) || (isstring(options.Units) && isscalar(options.Units)))
    error('metrics:sampleField:InvalidUnits', ...
        'Units must be a character vector or string scalar.');
end

% Meshgrid orientation keeps field rows aligned with y and columns with x.
x = linspace(bounds(1), bounds(2), gridSize(2));
y = linspace(bounds(3), bounds(4), gridSize(1));
[X, Y] = meshgrid(x, y);
queryPoints = [X(:), Y(:)];
% Evaluate in one batch so the metric owns point ordering, not the caller.
values = evaluator(queryPoints);

if ~(isnumeric(values) && isreal(values) && isvector(values) && ...
        numel(values) == size(queryPoints, 1) && all(isfinite(values(:))))
    error('metrics:sampleField:InvalidEvaluatorOutput', ...
        ['evaluator must return one finite real numeric value for each ', ...
         'query point.']);
end

% Preserve plotting metadata with the sampled numeric grid.
field = struct();
field.Name = options.Name;
field.Units = options.Units;
field.Bounds = bounds;
field.GridSize = gridSize;
field.X = X;
field.Y = Y;
field.Values = reshape(double(values), size(X));
field.ZeroLevel = 0;
field.SignConvention = 'negative-inside';
end

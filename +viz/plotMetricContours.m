function handles = plotMetricContours(field, varargin)
%PLOTMETRICCONTOURS Plot signed metric contours on an axes.
%
%   handles = viz.plotMetricContours(field)
%   handles = viz.plotMetricContours(field, 'Parent', ax, ...
%       'Levels', -4:0.5:4, 'ShowZeroContour', true)

validateField(field);

parser = inputParser;
parser.FunctionName = 'viz.plotMetricContours';
addParameter(parser, 'Parent', []);
addParameter(parser, 'Levels', []);
addParameter(parser, 'ShowZeroContour', true);
addParameter(parser, 'ShowColorbar', true);
parse(parser, varargin{:});
options = parser.Results;

validateLogicalOption(options.ShowZeroContour, 'ShowZeroContour');
validateLogicalOption(options.ShowColorbar, 'ShowColorbar');
levels = validateLevels(options.Levels);

if isempty(options.Parent)
    figureHandle = figure('Name', char(field.Name), 'Color', 'w');
    axesHandle = axes('Parent', figureHandle);
else
    if ~isgraphics(options.Parent, 'axes')
        error('viz:plotMetricContours:InvalidParent', ...
            'Parent must be an axes graphics object.');
    end
    axesHandle = options.Parent;
    figureHandle = ancestor(axesHandle, 'figure');
end

values = field.Values;
zeroLevel = field.ZeroLevel;
minimumValue = min(values(:));
maximumValue = max(values(:));

if isempty(levels)
    levels = unique(linspace(minimumValue, maximumValue, 11));
end

levelTolerance = 32 * eps(max(1, max(abs([levels(:); zeroLevel]))));
regularLevels = levels(abs(levels - zeroLevel) > levelTolerance);

wasHolding = ishold(axesHandle);
hold(axesHandle, 'on');

handles = struct();
handles.Figure = figureHandle;
handles.Axes = axesHandle;
handles.Contours = gobjects(0);
handles.ZeroContour = gobjects(0);
handles.Colorbar = gobjects(0);

hasVariation = minimumValue < maximumValue;
if hasVariation && ~isempty(regularLevels)
    [~, handles.Contours] = contour(axesHandle, field.X, field.Y, values, ...
        regularLevels, 'LineWidth', 0.75);
    set(handles.Contours, 'HandleVisibility', 'off');
end

if hasVariation && options.ShowZeroContour && minimumValue <= zeroLevel && ...
        zeroLevel <= maximumValue
    [~, handles.ZeroContour] = contour(axesHandle, field.X, field.Y, values, ...
        [zeroLevel, zeroLevel], 'LineColor', [0, 0, 0], 'LineWidth', 1.5);
    set(handles.ZeroContour, 'HandleVisibility', 'off');
end

colormap(axesHandle, signedDistanceColormap());
limit = max(abs([minimumValue - zeroLevel, maximumValue - zeroLevel]));
if limit > 0
    caxis(axesHandle, [zeroLevel - limit, zeroLevel + limit]);
end

if options.ShowColorbar
    handles.Colorbar = colorbar(axesHandle);
    if isempty(field.Units)
        handles.Colorbar.Label.String = char(field.Name);
    else
        handles.Colorbar.Label.String = sprintf('%s (%s)', ...
            char(field.Name), char(field.Units));
    end
end

if ~wasHolding
    hold(axesHandle, 'off');
end
end

function validateField(field)
requiredFields = {'Name', 'Units', 'X', 'Y', 'Values', 'ZeroLevel'};
if ~isstruct(field) || ~all(isfield(field, requiredFields))
    error('viz:plotMetricContours:InvalidField', ...
        'field must be a structure returned by metrics.sampleField.');
end

if ~(isnumeric(field.X) && isnumeric(field.Y) && isnumeric(field.Values) && ...
        isreal(field.X) && isreal(field.Y) && isreal(field.Values) && ...
        isequal(size(field.X), size(field.Y), size(field.Values)) && ...
        size(field.X, 1) >= 2 && size(field.X, 2) >= 2 && ...
        all(isfinite(field.X(:))) && all(isfinite(field.Y(:))) && ...
        all(isfinite(field.Values(:))))
    error('viz:plotMetricContours:InvalidField', ...
        'field X, Y, and Values must be matching finite grids of size at least 2-by-2.');
end

if ~(isnumeric(field.ZeroLevel) && isscalar(field.ZeroLevel) && ...
        isreal(field.ZeroLevel) && isfinite(field.ZeroLevel))
    error('viz:plotMetricContours:InvalidField', ...
        'field.ZeroLevel must be a finite real scalar.');
end

if ~(ischar(field.Name) || (isstring(field.Name) && isscalar(field.Name))) || ...
        ~(ischar(field.Units) || (isstring(field.Units) && isscalar(field.Units)))
    error('viz:plotMetricContours:InvalidField', ...
        'field Name and Units must be character vectors or string scalars.');
end
end

function levels = validateLevels(levels)
if isempty(levels)
    return;
end

if ~(isnumeric(levels) && isreal(levels) && isvector(levels) && ...
        all(isfinite(levels(:))) && all(diff(levels(:)) > 0))
    error('viz:plotMetricContours:InvalidLevels', ...
        'Levels must be a strictly increasing vector of finite real values.');
end

levels = double(levels(:).');
end

function validateLogicalOption(value, optionName)
if ~(islogical(value) && isscalar(value))
    error('viz:plotMetricContours:InvalidOption', ...
        '%s must be a logical scalar.', optionName);
end
end

function colors = signedDistanceColormap()
steps = 128;
negative = [linspace(0.10, 1.00, steps).', ...
    linspace(0.25, 1.00, steps).', ...
    linspace(0.80, 1.00, steps).'];
positive = [linspace(1.00, 0.80, steps).', ...
    linspace(1.00, 0.15, steps).', ...
    linspace(1.00, 0.10, steps).'];
colors = [negative; positive];
end

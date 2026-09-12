function manifest = saveSimulationFrames(result, varargin)
%SAVESIMULATIONFRAMES Export selected replay frames as PNG files.
%
%   manifest = viz.saveSimulationFrames(result) writes at most five evenly
%   spaced frames, including the first and final samples, below the repository
%   frames/<scenario>/<timestamp>/ directory. The simulation is not advanced.
%
%   FrameIndices selects exact logged sample indices. OutputRoot overrides the
%   default repository-local frames directory. Resolution is in DPI.

defaultOutputRoot = fullfile(projectRoot(), 'frames');
parser = inputParser;
parser.FunctionName = 'viz.saveSimulationFrames';
addParameter(parser, 'FrameIndices', []);
addParameter(parser, 'OutputRoot', defaultOutputRoot);
addParameter(parser, 'Resolution', 120);
addParameter(parser, 'ShowRays', false);
addParameter(parser, 'ShowHitPoints', true);
parse(parser, varargin{:});
options = parser.Results;

% Validate all inputs before creating a figure or output directory.
frameIndices = selectFrameIndices(options.FrameIndices, numel(result.Time));
outputRoot = validateOutputRoot(options.OutputRoot);
if ~(isnumeric(options.Resolution) && isreal(options.Resolution) && ...
        isscalar(options.Resolution) && isfinite(options.Resolution) && ...
        options.Resolution > 0 && options.Resolution == floor(options.Resolution))
    error('viz:saveSimulationFrames:InvalidOption', ...
        'Resolution must be a positive integer.');
end
if isempty(which('exportgraphics'))
    error('viz:saveSimulationFrames:MissingExportGraphics', ...
        'Static frame export requires exportgraphics (MATLAB R2020a or newer).');
end
% Reuse replay rendering while disabling playback and keeping state immutable.
handles = viz.animateSimulation(result, 'Visible', false, 'AutoPlay', false, ...
    'ShowRays', options.ShowRays, 'ShowHitPoints', options.ShowHitPoints);
figureCleanup = onCleanup(@() closeIfValid(handles.Figure));
outputDirectory = createOutputDirectory(outputRoot, result.Config.Name);
exportCompleted = false;
% Remove the output directory unless every requested frame exports successfully.
exportCleanup = onCleanup(@removeIncompleteExport);
filePaths = cell(numel(frameIndices), 1);
% Render and export only explicitly selected logged samples.
for index = 1:numel(frameIndices)
    frameIndex = frameIndices(index);
    handles.UpdateFrame(frameIndex);
    filePaths{index} = fullfile(outputDirectory, sprintf( ...
        'frame_%04d_t_%0.3fs.png', frameIndex, result.Time(frameIndex)));
    exportgraphics(handles.Figure, filePaths{index}, 'Resolution', options.Resolution);
end

manifest = struct( ...
    'Directory', outputDirectory, ...
    'FrameIndices', frameIndices, ...
    'Times', result.Time(frameIndices), ...
    'FilePaths', {filePaths});
exportCompleted = true;
clear exportCleanup figureCleanup;

    function removeIncompleteExport()
        if ~exportCompleted && exist(outputDirectory, 'dir')
            try
                rmdir(outputDirectory, 's');
            catch
                % Preserve the original export failure if cleanup also fails.
            end
        end
    end
end

function frameIndices = selectFrameIndices(requestedIndices, sampleCount)
%SELECTFRAMEINDICES Default to a compact, evenly spaced first-to-final selection.

if isempty(requestedIndices)
    frameCount = min(5, sampleCount);
    frameIndices = unique(round(linspace(1, sampleCount, frameCount))).';
    return;
end
if ~(isnumeric(requestedIndices) && isreal(requestedIndices) && ...
        isvector(requestedIndices) && all(isfinite(requestedIndices(:))) && ...
        all(requestedIndices(:) == floor(requestedIndices(:))) && ...
        all(requestedIndices(:) >= 1) && all(requestedIndices(:) <= sampleCount))
    error('viz:saveSimulationFrames:InvalidFrameIndices', ...
        'FrameIndices must contain valid integer sample indices.');
end
frameIndices = unique(double(requestedIndices(:)));
end

function outputRoot = validateOutputRoot(value)
if ~(ischar(value) || (isstring(value) && isscalar(value))) || isempty(strtrim(char(value)))
    error('viz:saveSimulationFrames:InvalidOutputRoot', ...
        'OutputRoot must be a nonempty character vector or string scalar.');
end
outputRoot = char(value);
end

function outputDirectory = createOutputDirectory(outputRoot, scenarioName)
%CREATEOUTPUTDIRECTORY Isolate each export transaction below a safe scenario name.

scenarioDirectory = fullfile(outputRoot, sanitizeScenarioName(scenarioName));
if ~exist(scenarioDirectory, 'dir')
    [created, message] = mkdir(scenarioDirectory);
    if ~created
        error('viz:saveSimulationFrames:CreateOutputDirectoryFailed', ...
            'Could not create frame output directory: %s', message);
    end
end
timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
outputDirectory = fullfile(scenarioDirectory, timestamp);
suffix = 2;
while exist(outputDirectory, 'dir')
    outputDirectory = fullfile(scenarioDirectory, sprintf('%s_%02d', timestamp, suffix));
    suffix = suffix + 1;
end
[created, message] = mkdir(outputDirectory);
if ~created
    error('viz:saveSimulationFrames:CreateOutputDirectoryFailed', ...
        'Could not create frame output directory: %s', message);
end
end

function name = sanitizeScenarioName(value)
if ~(ischar(value) || (isstring(value) && isscalar(value)))
    error('viz:saveSimulationFrames:InvalidScenarioName', ...
        'result.Config.Name must be a character vector or string scalar.');
end
name = lower(strtrim(char(value)));
name = regexprep(name, '[^a-z0-9]+', '-');
name = regexprep(name, '^-|-$', '');
if isempty(name)
    error('viz:saveSimulationFrames:InvalidScenarioName', ...
        'result.Config.Name must contain at least one letter or digit.');
end
end

function closeIfValid(figureHandle)
if isgraphics(figureHandle)
    close(figureHandle);
end
end

function root = projectRoot()
root = fileparts(fileparts(mfilename('fullpath')));
end

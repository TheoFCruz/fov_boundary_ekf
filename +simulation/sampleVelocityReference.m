function velocity = sampleVelocityReference(reference, time)
%SAMPLEVELOCITYREFERENCE Return the zero-order-held velocity at time.
%
%   The row at an exact breakpoint is active. The final row holds after its
%   breakpoint. Reference validity is normally established by
%   simulation.validateScenario.

if ~isstruct(reference) || ~isfield(reference, 'Times') || ...
        ~isfield(reference, 'Values')
    error('simulation:sampleVelocityReference:InvalidReference', ...
        'reference must contain Times and Values fields.');
end
if ~(isnumeric(time) && isscalar(time) && isreal(time) && isfinite(time))
    error('simulation:sampleVelocityReference:InvalidTime', ...
        'time must be a finite real scalar.');
end

times = reference.Times;
values = reference.Values;
if ~(isnumeric(times) && isreal(times) && isvector(times) && ~isempty(times) && ...
        all(isfinite(times(:))) && isnumeric(values) && isreal(values) && ...
        size(values, 1) == numel(times) && size(values, 2) == 3 && ...
        all(isfinite(values(:))) && all(diff(times(:)) > 0))
    error('simulation:sampleVelocityReference:InvalidReference', ...
        'reference Times and Values must form a finite increasing 3-axis schedule.');
end

times = double(times(:));
if time < times(1) - timeTolerance(times(1))
    error('simulation:sampleVelocityReference:BeforeStart', ...
        'time precedes the first reference breakpoint.');
end
index = find(double(time) >= times - timeTolerance(times), 1, 'last');
if isempty(index)
    index = 1;
end
velocity = reshape(double(values(index, :)), 1, 3);
end

function tolerance = timeTolerance(value)
tolerance = 64 * eps(max(1, abs(double(value))));
end

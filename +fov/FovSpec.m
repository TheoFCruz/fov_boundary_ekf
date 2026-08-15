classdef FovSpec
    %FOVSPEC Physical settings for a two-dimensional field of view.

    properties (SetAccess = private)
        MaxRange
        OpeningAngle
    end

    methods
        function obj = FovSpec(varargin)
            %FOVSPEC Construct an FOV specification.
            %
            %   spec = fov.FovSpec(maxRange, openingAngle)
            %   spec = fov.FovSpec(MaxRange=..., OpeningAngle=...)
            %
            % Angles are in radians. The opening angle may be at most 2*pi.

            if nargin == 2 && isnumeric(varargin{1}) && isnumeric(varargin{2})
                maxRange = varargin{1};
                openingAngle = varargin{2};
            else
                parser = inputParser;
                parser.FunctionName = 'fov.FovSpec';
                addParameter(parser, 'MaxRange', []);
                addParameter(parser, 'OpeningAngle', []);
                parse(parser, varargin{:});

                maxRange = parser.Results.MaxRange;
                openingAngle = parser.Results.OpeningAngle;
            end

            if ~(isnumeric(maxRange) && isscalar(maxRange) && ...
                    isreal(maxRange) && isfinite(maxRange) && maxRange > 0)
                error('fov:FovSpec:InvalidMaxRange', ...
                    'MaxRange must be a finite, positive numeric scalar.');
            end

            if ~(isnumeric(openingAngle) && isscalar(openingAngle) && ...
                    isreal(openingAngle) && isfinite(openingAngle) && ...
                    openingAngle > 0 && openingAngle <= 2*pi)
                error('fov:FovSpec:InvalidOpeningAngle', ...
                    'OpeningAngle must be in the interval (0, 2*pi].');
            end

            obj.MaxRange = double(maxRange);
            obj.OpeningAngle = double(openingAngle);
        end
    end
end

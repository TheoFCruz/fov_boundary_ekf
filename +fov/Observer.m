classdef Observer
    %OBSERVER Entity from which a two-dimensional FOV emanates.

    properties (SetAccess = private)
        Position
        Heading
        Fov
    end

    properties (Dependent)
        MaxRange
        OpeningAngle
    end

    methods
        function obj = Observer(varargin)
            %OBSERVER Construct an observer.
            %
            %   observer = fov.Observer(position, heading, ...
            %       maxRange, openingAngle)
            %   observer = fov.Observer(Position=..., Heading=..., ...
            %       MaxRange=..., OpeningAngle=...)
            %   observer = fov.Observer(Position=..., Heading=..., Fov=spec)
            %
            % Positions are [x, y] and angles are in radians.

            if nargin == 4 && isnumeric(varargin{1})
                position = varargin{1};
                heading = varargin{2};
                fovSpec = fov.FovSpec(varargin{3}, varargin{4});
            else
                parser = inputParser;
                parser.FunctionName = 'fov.Observer';
                addParameter(parser, 'Position', [0, 0]);
                addParameter(parser, 'Heading', 0);
                addParameter(parser, 'Fov', []);
                addParameter(parser, 'MaxRange', []);
                addParameter(parser, 'OpeningAngle', []);
                parse(parser, varargin{:});

                position = parser.Results.Position;
                heading = parser.Results.Heading;
                suppliedFov = parser.Results.Fov;
                maxRange = parser.Results.MaxRange;
                openingAngle = parser.Results.OpeningAngle;

                if isempty(suppliedFov)
                    if isempty(maxRange) || isempty(openingAngle)
                        error('fov:Observer:MissingFov', ...
                            'Provide Fov or both MaxRange and OpeningAngle.');
                    end
                    fovSpec = fov.FovSpec(maxRange, openingAngle);
                else
                    if ~isa(suppliedFov, 'fov.FovSpec')
                        error('fov:Observer:InvalidFov', ...
                            'Fov must be an fov.FovSpec object.');
                    end
                    if ~isempty(maxRange) || ~isempty(openingAngle)
                        error('fov:Observer:ConflictingFov', ...
                            'Do not combine Fov with MaxRange or OpeningAngle.');
                    end
                    fovSpec = suppliedFov;
                end
            end

            if ~(isnumeric(position) && isreal(position) && ...
                    numel(position) == 2 && all(isfinite(position(:))))
                error('fov:Observer:InvalidPosition', ...
                    'Position must contain two finite real numeric values.');
            end

            if ~(isnumeric(heading) && isscalar(heading) && ...
                    isreal(heading) && isfinite(heading))
                error('fov:Observer:InvalidHeading', ...
                    'Heading must be a finite real numeric scalar.');
            end

            obj.Position = reshape(double(position), 1, 2);
            obj.Heading = double(heading);
            obj.Fov = fovSpec;
        end

        function value = get.MaxRange(obj)
            value = obj.Fov.MaxRange;
        end

        function value = get.OpeningAngle(obj)
            value = obj.Fov.OpeningAngle;
        end
    end
end

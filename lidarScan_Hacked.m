classdef lidarScan
%lidarScan Create object for storing 2D LiDAR scan
%   SCAN = lidarScan(RANGES, ANGLES) creates a lidarScan object where
%   the input RANGES and ANGLES are specified as vectors of equal
%   length. The RANGES are specified in meters and ANGLES in radians.
%   The angles are measured counter-clockwise around the positive Z axis.
%
%   SCAN = lidarScan(CART) initializes the object with Cartesian
%   coordinates. CART is specified as an N-by-2 matrix, with the first
%   column containing X coordinates and the second column containing Y
%   coordinates (in meters). In the right-handed LiDAR coordinate frame,
%   X is forward and Y is to the left.
%
%   lidarScan properties:
%      Ranges     - Range values of laser scan in meters (read-only)
%      Angles     - Angular values in radians (read-only)
%      Cartesian  - Matrix of Cartesian coordinates (read-only)
%      Count      - Number of ranges and angles in the scan (read-only)
%
%   lidarScan methods:
%      plot              - Plot LiDAR scan data
%      removeInvalidData - Remove invalid range and angle data
%      transformScan     - Transform laser scan based on relative pose
%
%   Example:
%
%         % Use vectors of ranges and angles
%         ranges = [nan 1 2 3 inf 4 5 nan 6 7 8 0.1 inf 21];
%         angles = linspace(-pi/2, pi/2, numel(ranges));
%
%         scan = lidarScan(ranges, angles)
%
%         minRange = 0.1;
%         maxRange = 4.0;
%
%         % Remove invalid points for refScan (inf/nan + out of range limits)
%         validScan = removeInvalidData(scan, 'RangeLimits', [minRange maxRange])
%
%         % Visualize the scan
%         figure
%         plot(scan)
%         hold on
%         plot(validScan)
%
%   See also matchScans, monteCarloLocalization.

%   Copyright 2017-2019 The MathWorks, Inc.

%#codegen

    properties (Constant, Hidden)
        %MaxCount - Maximum allowed number of ranges and angles in the scan during codegen
        %   When used with the ERT code generation target, an INF upper
        %   bound is not allowed, thus using a high constant number instead.
        %   Also this case is applicable with "dynamic memory allocation"
        %   turned off in models containing the Simulink Algorithm blocks
        %   using lidarScan
        MaxCount = 4000
    end

    properties (Dependent, SetAccess = public)
        %Ranges - Range values of laser scan in meters
        %   Each range is associated with the polar angle of the same index
        %   in Angles.
        Ranges

        %Angles - Angular values in radians
        %   Each angular value corresponds to a range with the same index
        %   in Ranges.
        %   The angles are measured counter-clockwise around the positive Z axis
        %   and will be in radians.
        Angles

        %Cartesian - Matrix of Cartesian coordinates
        %   In the right-handed LiDAR coordinate frame, X is forward and
        %   Y is to the left.
        Cartesian

        %Count - Number of ranges and angles in the scan
        Count
    end

    properties (Access = {?lidarScan, ?matlab.unittest.TestCase})
        %InternalRanges - Internal storage for dependent Ranges property
        InternalRanges

        %InternalAngles - Internal storage for dependent Angles property
        InternalAngles

        %ContainsOnlyFiniteData - Internal flag for tracking finite data
        ContainsOnlyFiniteData = false
    end

    methods
        function obj = lidarScan(varargin)
        %lidarScan Construct lidarScan object
        %   The constructor validates the data on ranges and angles inputs
        %   and ensure the dimensions are correct.

            narginchk(1,2);

            coder.varsize('validRanges', [lidarScan.MaxCount 1], [1 0]);
            coder.varsize('validAngles', [lidarScan.MaxCount 1], [1 0]);

            if nargin == 2
                % ranges and angles are passed as input

                aRanges = varargin{1};
                aAngles = varargin{2};
                [validRanges, validAngles] = robotics.internal.validation.validateLaserScan(aRanges, aAngles, 'lidarScan', 'ranges', 'angles', true);

            elseif nargin == 1
                % Cartesian coordinates [x y] are passed as input

                aCart = varargin{1};
                if isempty(aCart)
                    validateattributes(aCart, {'single', 'double'}, {}, 'lidarScan', 'cart');
                    aAngles = [];
                    aRanges = [];
                else
                    validateattributes(aCart, {'single', 'double'}, {'nonempty', 'real', 'size', [NaN,2]}, 'lidarScan', 'cart');
                    [aAngles, aRanges] = cart2pol(aCart(:,1), aCart(:,2));
                end

                % Validate if the ranges and angles are as expected
                % Use-case: if someone has Cartesian with 'inf/nan' present,
                % cart2pol will return aAngles and aRanges with inf/nan
                [validRanges, validAngles] = robotics.internal.validation.validateLaserScan(aRanges, aAngles, 'lidarScan', 'ranges', 'angles', true);
            end

            obj.InternalRanges = validRanges;

            % Wrap the angles to [-pi, pi]
            obj.InternalAngles = robotics.internal.wrapToPi(validAngles);
        end
        
        function count = get.Count(obj)
        %get.Count Return the number of stored ranges and angles
            count = numel(obj.InternalRanges);
        end
    end

    %% Dependent property SETTERS
    methods
%         function obj = set.OldPropName(obj,val)
%             obj.NewPropName = val;
%         end
        function cart = set.Cartesian(cart,val)
            [cart.InternalAngles, cart.InternalRanges] = cart2pol(val(:, 1), val(:, 2));
        end

        function ranges = set.Ranges(ranges,val)
            ranges.InternalRanges = val;
        end

        function angles = set.Angles(angles,val)
            angles.InternalAngles = val;
        end
    end
    
    %% Dependent property GETTERS
    methods
        function cart = get.Cartesian(obj)
        %get.Cartesian Return Cartesian points corresponding to polar data
            cart = zeros(numel(obj.InternalRanges), 2, class(obj.InternalRanges));
            [cart(:, 1), cart(:, 2)] = pol2cart(obj.InternalAngles, obj.InternalRanges);
        end

        function ranges = get.Ranges(obj)
        %get.Ranges Return ranges
            ranges = obj.InternalRanges;
        end

        function angles = get.Angles(obj)
        %get.Angles Return angles
            angles = obj.InternalAngles;
        end
    end

    %% User-facing class methods
    methods
        function plotHandles = plot(obj, varargin)
        %PLOT Plot LiDAR scan data
        %   PLOT(SCAN) creates a point plot of the lidarScan object SCAN
        %   in the current axes (or creates a new one). The scan is
        %   shown in Cartesian (XY) coordinates and the axes are
        %   automatically scaled to the maximum range in the SCAN
        %   object.
        %
        %   PLOT(___, Name, Value) allows the specification of
        %   optional name/value pairs to control the plotting.
        %   Potential name/value pairs are:
        %      'Parent'       -  specifies the parent axes in which the LiDAR scan
        %                        should be drawn. By default, the scan is
        %                        plotted in the currently active axes.
        %
        %   PLOTHANDLES = PLOT(___) returns a column vector of lineseries
        %   handles, PLOTHANDLES, for the laser scan. Use PLOTHANDLES to
        %   modify properties of the lineseries after it is created.

        % If called in code generation, throw incompatibility error
            coder.internal.errorIf(~coder.target('MATLAB'), 'shared_robotics:robotcore:laserscan:GraphicsSupportCodegen', 'plot');

            % Only plot valid data
            scanObj = obj.removeInvalidData;

            defaults.Parent = [];
            scale = 1.1;
            if scanObj.Count == 0
                defaults.MaximumRange = scale;
            else
                defaults.MaximumRange = scale * max(scanObj.InternalRanges);
            end

            lineHandles = robotics.utils.internal.plotScan(metaclass(obj), ...
                                                           scanObj.Cartesian, scanObj.InternalAngles, defaults, varargin{:});

            % Return plot handle
            if nargout > 0
                plotHandles = lineHandles;
            end
        end

        function objOut = removeInvalidData(objIn, varargin)
        %removeInvalidData Remove invalid range and angle data
        %   VALIDSCAN = removeInvalidData(SCAN) removes the Inf/NaN
        %   values from the ranges, and corresponding angles of the
        %   lidarScan object, SCAN. The function returns a new
        %   lidarScan object, VALIDSCAN, containing only valid points.
        %
        %   VALIDSCAN = removeInvalidData(SCAN, Name, Value) provides
        %   additional options specified by one or more Name,Value pair
        %   arguments. Name must appear inside single quotes (''). You
        %   can specify several name-value pair arguments in any order
        %   as Name1, Value1, ..., NameN, ValueN:
        %
        %   'RangeLimits' - A 2-element vector specifying the minimum and
        %                   maximum valid range [minRange maxRange] in meters.
        %   'AngleLimits' - A 2-element vector specifying the minimum and
        %                   maximum valid angle [minAngle maxAngle] in radians.

        % Find all the indices which are NOT inf/nan

            if objIn.ContainsOnlyFiniteData && nargin == 1
                objOut = objIn;
                return;
            end

            if ~objIn.ContainsOnlyFiniteData
                validIndices = isfinite(objIn.InternalRanges) & isfinite(objIn.InternalAngles);
                objIn.ContainsOnlyFiniteData = true;
            else
                validIndices = true(size(objIn.InternalRanges));
            end

            % Defaults for range and angle limits and corresponding logical
            defaultRangeLimits = cast([0 inf], 'like', objIn.Ranges);
            defaultAngleLimits = cast([-pi, pi], 'like', objIn.Angles);
            validRangeLimitIndices = true(size(objIn.InternalRanges));
            validAngleLimitIndices = validRangeLimitIndices;

            if nargin > 1
                % Pick default range limits for codegen to succeed when only char
                % vector is passed without any arguments - this should fail
                % during MEX run

                % Parse input
                parameterNames = {'RangeLimits', 'AngleLimits'};
                defaultValuesForParameters = {defaultRangeLimits, defaultAngleLimits};
                parser = robotics.core.internal.NameValueParser(parameterNames,defaultValuesForParameters);
                parse(parser, varargin{:});

                % Extract parsed inputs
                parsedRangeLimits = parameterValue(parser, parameterNames{1});
                parsedAngleLimits = parameterValue(parser, parameterNames{2});

                % Validate the limits provided by the user
                objIn.validateRangeLimits(parsedRangeLimits, 'removeInvalidData', parameterNames{1});
                objIn.validateAngleLimits(parsedAngleLimits, 'removeInvalidData', parameterNames{2});

                % Find indices which do not meet the criteria
                if ~isequal(parsedRangeLimits, defaultRangeLimits)
                    validRangeLimitIndices = (objIn.InternalRanges >= parsedRangeLimits(1)) & (objIn.InternalRanges <= parsedRangeLimits(2));
                end

                if ~isequal(parsedAngleLimits, defaultAngleLimits)
                    % Wrap the input angle limits to [-pi, pi]
                    parsedAngleLimits = sort(robotics.internal.wrapToPi(parsedAngleLimits));

                    validAngleLimitIndices = (objIn.InternalAngles >= parsedAngleLimits(1)) & (objIn.InternalAngles <= parsedAngleLimits(2));
                end
            end

            % Remove indices which do not meet the criteria
            validIdx = validIndices & validRangeLimitIndices & validAngleLimitIndices;
            if ~all(validIdx)
                % Extract valid points only when needed
                objOut = extractValidData(objIn, validIdx);
                objOut.ContainsOnlyFiniteData = true;
            else
                objOut = objIn;
            end
        end

        function transScan = transformScan(obj, relPose)
        %transformScan Transform laser scan based on relative pose
        %   TRANSSCAN = transformScan(SCAN, RELPOSE) transforms the laser scan
        %   given by the lidarScan object, SCAN. The translation and rotation are
        %   defined in the relative pose, RELPOSE. The transformed laser scan is
        %   returned in TRANSSCAN.
        %
        %   RELPOSE is a 3-element vector, [x y theta], representing the relative pose
        %   that is used to transform the scan. [x y] is the translation
        %   (in meters) and [theta] is the rotation (in radians).
        %
        %   Example:
        %       % Example laser scan data input
        %       refRanges = 5 * ones(1, 300);
        %       refAngles = linspace(-pi/2, pi/2, 300);
        %       refScan = lidarScan(refRanges, refAngles);
        %
        %       % Translate laser scan by an (x,y) offset of (0.5, 0.2)
        %       translScan = transformScan(refScan, [0.5, 0.2, 0]);
        %
        %       % Rotate laser scan by 20 degrees
        %       rotScan = transformScan(refScan, [0, 0, deg2rad(20)]);

            validPose = robotics.internal.validation.validateMobilePose(...
                relPose, 'transformScan', 'relPose');

            transScan = nav.algs.internal.transformScan(obj, validPose);
        end
    end

    methods (Access = private)
        function objOut = extractValidData(objIn, validIndices)
        %extractValidData Return a new lidarScan object based on valid indices
        %   This method ensure the dimension resizing is codegen compatible.

        % For enabling codegen
            coder.varsize('newRanges');
            coder.varsize('newAngles');

            newRanges = objIn.InternalRanges;
            newAngles = objIn.InternalAngles;
            newRanges(~validIndices) = [];
            newAngles(~validIndices) = [];

            % Construct new lidarScan object
            % Note that it is possible for ranges and angles to be empty.
            objOut = lidarScan(newRanges, newAngles);
        end
    end

    methods (Static, Access = private)
        function validateRangeLimits(aRangeLimits, fcnName, varName)
        %validateRangeLimits Validate RangeLimits user inputs
        %   Ensures the specified bounds are valid: dimension,
        %   nondecreasing, nonnegative checks.
            validateattributes(aRangeLimits, {'numeric'}, ...
                               {'vector', 'numel', 2, 'nonnan', 'nonnegative', 'nondecreasing', 'real'}, ...
                               fcnName, varName);

        end

        function validateAngleLimits(aAngleLimits, fcnName, varName)
        %validateAngleLimits Validate AngleLimits user inputs
            validateattributes(aAngleLimits, {'numeric'}, ...
                               {'vector', 'numel', 2, 'nonnan', 'nondecreasing', 'real'}, ...
                               fcnName, varName);
        end
    end
end
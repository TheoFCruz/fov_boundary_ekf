function endpoints = transportEndpoints(angles, ranges, rangeVariances, deltaP, deltaPsi)
%TRANSPORTENDPOINTS Move old beam endpoints into the new sensor frame (eq. 1).
%
%   endpoints = estimation.transport.transportEndpoints( ...
%       angles, ranges, rangeVariances, deltaP, deltaPsi)
%
% Each old endpoint ranges(j) * u(angles(j)) becomes
% p'_j = R(-deltaPsi) * (p_j - deltaP), with new range Rho and bearing Phi
% (wrapped to [-pi, pi)). A no-return beam's endpoint is its cap at MaxRange:
% transporting it gives a bearing for the bracket search, not a surface.
%
% RangeVariance is each old variance projected onto the new range,
% cos(alpha_j)^2 * Sigma_jj, with the tilt alpha_j = theta_j - deltaPsi -
% phi_j (eq. 4); it feeds the uncertainty term of the eq. 7 jump test.
% Tilt is alpha_j itself, kept for the eq. 4-5 Jacobian
% (estimation.transport.propagateCovariance).
%
% Ranges below 1 mm are treated as 1 mm. A point at the sensor has no
% bearing (atan2(0, 0)), and eq. 4's dphi/dmu = sin(alpha) / rho would
% divide by zero; noisy returns clipped to 0 next to a post produce exactly
% that.

minimumRange = 1e-3;
oldPoints = max(ranges, minimumRange) .* [cos(angles), sin(angles)];
rotation = [cos(-deltaPsi), -sin(-deltaPsi); sin(-deltaPsi), cos(-deltaPsi)];
newPoints = (oldPoints - deltaP.') * rotation.';

endpoints = struct();
endpoints.Rho = hypot(newPoints(:, 1), newPoints(:, 2));
endpoints.Phi = atan2(newPoints(:, 2), newPoints(:, 1));
endpoints.Tilt = mod(angles - deltaPsi - endpoints.Phi + pi, 2 * pi) - pi;
endpoints.RangeVariance = (cos(endpoints.Tilt) .^ 2) .* rangeVariances;
end

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

oldPoints = ranges .* [cos(angles), sin(angles)];
rotation = [cos(-deltaPsi), -sin(-deltaPsi); sin(-deltaPsi), cos(-deltaPsi)];
newPoints = (oldPoints - deltaP.') * rotation.';

endpoints = struct();
endpoints.Rho = hypot(newPoints(:, 1), newPoints(:, 2));
endpoints.Phi = atan2(newPoints(:, 2), newPoints(:, 1));
tilt = mod(angles - deltaPsi - endpoints.Phi + pi, 2 * pi) - pi;
endpoints.RangeVariance = (cos(tilt) .^ 2) .* rangeVariances;
end

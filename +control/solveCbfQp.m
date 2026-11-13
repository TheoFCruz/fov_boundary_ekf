function [input, qp] = solveCbfQp(coefficient, rightHandSide, nominal, config, options)
%SOLVECBFQP Minimally modify a nominal input subject to one CBF constraint.
%
%   [input, qp] = control.solveCbfQp(coefficient, rightHandSide, nominal, config, options)
%
% Solves, with quadprog,
%
%     min over u, delta   (u - u0)' * W * (u - u0) + lambda * delta^2
%     subject to          coefficient * u + delta >= rightHandSide
%                         InputLower <= u <= InputUpper,   delta >= 0
%
% where u0 is the nominal input clipped to the bounds and delta is the slack
% that keeps the problem feasible. For the CBF condition hdot + gamma*h >= 0
% the policy passes coefficient = dh/du (1-by-3) and
% rightHandSide = -gamma*h - dh/dt. config supplies InputLower and InputUpper
% (3-by-1), InputWeights (W) and SlackPenalty (lambda); options are quadprog
% options built once by the caller.
%
% input is the 1-by-3 applied input. qp records Status, NominalInput (u0),
% Slack, ConstraintResidual, ActiveBounds, ExitFlag and Iterations. Status is
%   "nominal-feasible"        u0 already satisfies the constraint;
%   "cbf-corrected"           the constraint is met within the bounds;
%   "relaxed-with-slack"      no bounded input meets it, slack takes the rest;
%   "zero-control-authority"  bounded inputs cannot change coefficient * u;
%   "qp-failure-fallback"     quadprog failed or returned an unusable result.
% The last two return u0, never a stop command, so a nonzero nominal does not
% cause stop-go motion.

nominal = reshape(double(nominal), 3, 1);
clippedNominal = min(max(nominal, config.InputLower), config.InputUpper);
input = clippedNominal.';
qp = struct( ...
    'Status', "not-computed", ...
    'NominalInput', clippedNominal, ...
    'Slack', NaN, ...
    'ConstraintResidual', NaN, ...
    'ActiveBounds', false(3, 1), ...
    'ExitFlag', NaN, ...
    'Iterations', NaN);

% Range of coefficient * u over the input box. With no range, no bounded
% input can affect the barrier, so the QP is skipped.
lowerTerms = coefficient.' .* config.InputLower;
upperTerms = coefficient.' .* config.InputUpper;
minimumContribution = sum(min(lowerTerms, upperTerms));
maximumContribution = sum(max(lowerTerms, upperTerms));
if norm(coefficient) <= sqrt(eps) ...
        || maximumContribution - minimumContribution <= sqrt(eps)
    qp.Status = "zero-control-authority";
    return;
end

% Decision vector [u; delta]. Start quadprog from u0 with only the slack the
% constraint requires there, which is always feasible.
initialSlack = max(0, rightHandSide - coefficient * clippedNominal);
initialPoint = [clippedNominal; initialSlack];
H = blkdiag(config.InputWeights, config.SlackPenalty);
f = [-config.InputWeights * clippedNominal; 0];
A = [-coefficient, -1];
b = -rightHandSide;
lowerBound = [config.InputLower; 0];
upperBound = [config.InputUpper; Inf];

% Any solver error or non-finite result falls back to u0.
try
    [solution, ~, exitFlag, output] = quadprog( ...
        H, f, A, b, [], [], lowerBound, upperBound, initialPoint, options);
    qp.ExitFlag = exitFlag;
    qp.Iterations = output.iterations;
    isUsable = isscalar(exitFlag) ...
        && isfinite(exitFlag) ...
        && exitFlag > 0 ...
        && numel(solution) == 4 ...
        && all(isfinite(solution)) ...
        && isfinite(qp.Iterations);
catch
    isUsable = false;
end
if ~isUsable
    qp.Status = "qp-failure-fallback";
    return;
end

% Classify from the constraint geometry, not the slack magnitude: slack is
% reported numerically but never used for the status.
controlInput = solution(1:3);
input = controlInput.';
qp.Slack = solution(4);
qp.ConstraintResidual = coefficient * controlInput + qp.Slack - rightHandSide;
qp.ActiveBounds = abs(controlInput - config.InputLower) <= sqrt(eps) ...
    | abs(controlInput - config.InputUpper) <= sqrt(eps);
if coefficient * clippedNominal >= rightHandSide - sqrt(eps)
    qp.Status = "nominal-feasible";
elseif maximumContribution < rightHandSide - sqrt(eps)
    qp.Status = "relaxed-with-slack";
else
    qp.Status = "cbf-corrected";
end
end

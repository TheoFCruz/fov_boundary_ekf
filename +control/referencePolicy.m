function [input, state, diagnostics] = referencePolicy(~, reference, state)
%REFERENCEPOLICY Return the prescribed observer body-frame velocity.

if ~(isnumeric(reference) && isreal(reference) && numel(reference) == 3 && ...
        all(isfinite(reference(:))))
    error('control:referencePolicy:InvalidReference', ...
        'reference must contain three finite real numeric values.');
end
input = reshape(double(reference), 1, 3);
diagnostics = struct('Method', "reference-policy");
end

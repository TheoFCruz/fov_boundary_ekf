function velocity = sampleVelocityReference(reference, time)
%SAMPLEVELOCITYREFERENCE Return the zero-order-held velocity at time.
%
%   The row at an exact breakpoint is active. Scenario validation establishes
%   the schedule before the runner samples it.

index = find(time >= reference.Times - 64 * eps(max(1, abs(reference.Times))), ...
    1, 'last');
velocity = reference.Values(index, :);
end

function flag = check_constraints(constraints, t, e)
%CHECK_CONSTRAINTS check if the given constraints are satisfied 

if ~isduration(t)
    error('t should be a duration')
end

const_idx = find(constraints.Time > t, 1, 'first');
flag = e > constraints.energy(const_idx);

end
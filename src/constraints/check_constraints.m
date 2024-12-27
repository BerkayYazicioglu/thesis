 function flag = check_constraints(constraints, t, e)
%CHECK_CONSTRAINTS check if the given constraints are satisfied 

const_idx = find(constraints.Time > t, 1, 'first');
flag = e > constraints.energy(const_idx);

 end
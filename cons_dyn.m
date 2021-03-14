function f_dyn = cons_dyn(A,B)
global x u H;

    f_dyn = [];

    for k = 2:H % First step is already constrained by initial condition
        f_dyn = [f_dyn, x{k} == A*x{k-1} + B*u{k-1}];

    end

end
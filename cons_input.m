function f_input = cons_input(u_limit)

global u H;

    f_input = [];

    for k = 1:H-1 % For a system with 2 inputs
        f_input = [f_input, -u_limit * ones(2,1)<= u{k} <= u_limit * ones(2,1) ];

    end


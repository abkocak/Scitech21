function [f_init] =cons_init(x0)
global x;

f_init= [x{1} == x0];

end
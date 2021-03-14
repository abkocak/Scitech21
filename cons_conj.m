function [z,f_and] = cons_conj(formula,list)   

global z H;
ms = length(list);

z_and = create_z(formula);

f_and = [];
for k = 1 : H
    sums = 0;
    for i = 1:length(list)
        f_list = list{i};
        f_and = [f_and , z(k).(formula) <= z(k).(f_list)];
        sums = sums + z(k).(f_list);    
    end
        f_and = [f_and, z(k).(formula) >= 1 - ms + sums];
end

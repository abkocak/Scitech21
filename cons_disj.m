function [z,f_or] = cons_disj(formula,list)   

global z H


z_or = create_z(formula);

f_or=[];
for k = 1 : H
    sums = 0;
    for i = 1:length(list)
        f_list = list{i};
        f_or = [f_or , z(k).(formula) >= z(k).(f_list)];
        sums = sums + z(k).(f_list);    
    end
        f_or = [f_or, z(k).(formula) <= sums];
end

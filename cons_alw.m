function [z,f_alw] = cons_alw(phi_out,phi_in,interval)   

global z H
ms = length(interval);

z_alw = create_z(phi_out);

f_alw = [];
for k = 1 : H-interval(end)
    sums = 0;
    for tau = k+interval(1) : k+interval(end)
        f_alw = [f_alw , z(k).(phi_out) <= z(tau).(phi_in)];
        sums = sums + z(tau).(phi_in);    
    end
        f_alw = [f_alw, z(k).(phi_out) >= 1 - ms + sums];
end

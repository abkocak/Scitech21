function [z,f_evn] = cons_evn(phi_out,phi_in,interval)   

global z H

z_evn = create_z(phi_out);

f_evn = [];
for k = 1 : H-interval(end)
    sums = 0;
    for tau = k+interval(1) : k+interval(end)
        f_evn = [f_evn , z(k).(phi_out) >= z(tau).(phi_in)];
        sums = sums + z(tau).(phi_in);    
    end
        f_evn = [f_evn, z(k).(phi_out) <= sums];
end

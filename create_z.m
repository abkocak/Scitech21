function [z,f_pred] = create_z(formula)
global x x_agents z H big_M delta r pred p_stat

for k = 1:H
    z(k).(formula) = binvar(1);  % Create z for that formula or predicate
end

f_pred=[];

if contains(formula,'pred_') % If it is a predicate
ii = str2num(extractAfter(formula,'pred_')); % Extract predicate number

if ii<p_stat+1 % Static/Uncoupled Predicates on Regions of interests (p_stat)

    for k = 1:H
          f_pred = [f_pred,...
          pred(ii,1)*x{k}(pred(ii,2))+pred(ii,3)+delta{k}(ii)...
          >= big_M * (z(k).(formula) - 1)];
        
          f_pred = [f_pred,...
          pred(ii,1)*x{k}(pred(ii,2))+pred(ii,3)+delta{k}(ii)...
          <= big_M * z(k).(formula)];
    end
    
else % Relative constraints coupled with other agents (p_dyn)
 
    for k = 1:H
          f_pred = [f_pred,...
          pred(ii,1)*(x{k}(pred(ii,2))-x_agents{pred(ii,3)}(pred(ii,2),k))+r(ii)+delta{k}(ii)...
          >= big_M * (z(k).(formula) - 1)];
          
          f_pred = [f_pred,...
          pred(ii,1)*(x{k}(pred(ii,2))-x_agents{pred(ii,3)}(pred(ii,2),k))+r(ii)+delta{k}(ii)...
          <= big_M * z(k).(formula)];
    end
end

% Note: All predicates are relaxed by the variable delta(t)

%     Example x(k) < -3
%             for k = 1:H
%                 f_pred = [f_pred, -x{k}(1)-3+delta(ii) >= big_M * (z(k).(formula) - 1)];
%                 f_pred = [f_pred, -x{k}(1)-3+delta(ii) <= big_M * z(k).(formula)];
%             end
%     
%      Example y(k) > +1
%     
%             for k = 1:H
%                 f_pred = [f_pred, x{k}(2)-1+delta(ii) >= big_M * (z(k).(formula) - 1)];
%                 f_pred = [f_pred, x{k}(2)-1+delta(ii) <= big_M * z(k).(formula)];
%             end


end
end
        
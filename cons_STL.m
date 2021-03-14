%% Function to encode STL formula constraints
% Input: agent index
% Output: Stacked STL constraints (Predicate bigM's and Boolean connections)

function [f_STL] = cons_STL(i_agent)

global z H hrz dt big_M pred delta r rmin rmax boxes obstacles p_stat p_dyn k n_agents

%% Definition of predicates

big_M=10000; % Big M to be used in predicate constraints

% Predicate matrix: [sign (geq:+1 or leq:-1), State (x:1 or y:2), predicate threshold value]
% For example if ith predicate is x \leq 5, then predi(i,:) = [-1,1,5]
%             if jth predicate is x \geq 5, then predi(j,:) = [1,1,-5]

sign = [];
states = [];
constraints = [];

for i=1:p_stat/4
sign = [sign -1 1 -1 1];
states = [states 1 1 2 2];
constraints = [constraints boxes{i_agent}(1+4*(i-1)) -boxes{i_agent}(2+4*(i-1))...
                           boxes{i_agent}(3+4*(i-1)) -boxes{i_agent}(4+4*(i-1))];
end

r = zeros(1,p_stat);

% Relative constraint parameters
if k > 1 % First iteration contains no coupling!
for i=1:n_agents
    if i ~= i_agent
    sign = [sign -1 1 -1 1 -1 1 -1 1];
    states = [states 1 1 2 2 1 1 2 2];
    constraints = [constraints i i i i i i i i];
    r = [r -rmin -rmin -rmin -rmin rmax rmax rmax rmax];
    end
end
end

p_dyn = size(sign,2)-p_stat; % Number of Pairwise Coupled predicates

% Combine all predicates
for i=1:p_stat+p_dyn
    predi(i,:)=[sign(i) states(i) constraints(i)];
end
pred=predi;

% Define predicate relaxation variable
%delta = sdpvar(size(pred,1),1); % If fixed delta over the time was used
delta = sdpvar(repmat(size(pred,1),1,H),repmat(1,1,H)); % We use time dependent delta

% Create predicate MILP constraints (bigM constraints)
f_bigM = [];
for i=1:size(pred,1)
    [z,f_predi] = create_z(sprintf('pred_%d',i));
    f_bigM = [f_bigM, f_predi];
end
f_STL = [f_bigM]; % Start to append STL constraints, to be continued...

%% Connection of predicates and temporal operators: Global STL Formula

% Part-time objectives (goal regions)
ands_pt={}; 
for i=1:size(boxes{i_agent},2)/4-size(obstacles,2)/4 % For each goal region, no obstacles yet
eventglob_in_i = sprintf('evglob_inner_%d',i);
[z,f1]=cons_conj(eventglob_in_i,{sprintf('pred_%d',1+4*(i-1)),... % Each region has 4 line bounds
                                sprintf('pred_%d',2+4*(i-1)),...
                                sprintf('pred_%d',3+4*(i-1)),...
                                sprintf('pred_%d',4+4*(i-1))}); %Green regions (3 regions in paper)
                      
eventglob_out_i = sprintf('evglob_outer_%d',i);
[z,f2]=cons_alw(eventglob_out_i,eventglob_in_i,[0/dt:2/dt]); % Inner Globally

eventglob_i = sprintf('evglob_%d',i);
[z,f3]=cons_evn(eventglob_i,eventglob_out_i,[20/dt:hrz-2/dt]); % Outer Eventually

f_STL = [f_STL,f1,f2,f3]; % Append the goal region constraints of the STL formula, to be continued...

ands_pt = [ands_pt eventglob_i]; % Same Eventually-Globally strings is appended for all regions
end

% Full-time objectives (Obstacle-Collision avoidance and coherence)
ands_ft={}; 
kk = p_stat-size(obstacles,2)+1; % First Obstacle predicate index among all predicates
for i=1:size(obstacles,2)/4 % For each obstacle
or_obs_i = sprintf('or_obs%d',i);
[z,fobs]=cons_disj(or_obs_i,{sprintf('pred_%d',kk+4*(i-1)),... % Each obstacle has 4 line bounds
                             sprintf('pred_%d',kk+1+4*(i-1)),...
                             sprintf('pred_%d',kk+2+4*(i-1)),...
                             sprintf('pred_%d',kk+3+4*(i-1))}); % Static obstacles (2 obs in paper)

f_STL = [f_STL,fobs]; % Append obstacle constraints (no mention on globally yet)

ands_ft = [ands_ft or_obs_i]; % Append formula strings for obstacles
end                        
    
if k > 1 % If coupling between agents is present (all iterations except the first)
    j=0;
    for i=1:n_agents
        if i~= i_agent
        j=j+1;
        
        % Collision avoidance constraints (similar encoding with the obstacles)
        
        or_i = sprintf('or_%d',i);
        
        pred_or_1 = sprintf('pred_%d',p_stat+1+8*(j-1));
        pred_or_2 = sprintf('pred_%d',p_stat+2+8*(j-1));
        pred_or_3 = sprintf('pred_%d',p_stat+3+8*(j-1));
        pred_or_4 = sprintf('pred_%d',p_stat+4+8*(j-1));
        
        [z,f_or]=cons_disj(or_i,{pred_or_1,pred_or_2,pred_or_3,pred_or_4}); % Inner fit box
        
        f_STL = [f_STL,f_or]; % Append disjunction constraints, to be continued...
        
        % Coherence constraints (similar encoding with the goal regions)

        pred_and_1 = sprintf('pred_%d',p_stat+5+8*(j-1));
        pred_and_2 = sprintf('pred_%d',p_stat+6+8*(j-1));
        pred_and_3 = sprintf('pred_%d',p_stat+7+8*(j-1));
        pred_and_4 = sprintf('pred_%d',p_stat+8+8*(j-1));
        
        % Append coupled constraints with full-time ones (Coll.avoidance "AND" 4 Coherence Preds)
        ands_ft = [ands_ft or_i pred_and_1 pred_and_2 pred_and_3 pred_and_4];        
        end
     end
end

[z,fand_ft]=cons_conj('globally_inner_ft',ands_ft); % Combine all full-time constraints
   
[z,falw_ft]=cons_alw('phi_ft','globally_inner_ft',[0/dt:hrz/dt]); %Full-time constraints globally valid

[z,fphi]=cons_conj('phi',[ands_pt 'phi_ft']); % Combine Part-time (F_G_) and full-time (G_)

f_STL = [f_STL,fand_ft,falw_ft,fphi]; % Append all the remaining connections

end

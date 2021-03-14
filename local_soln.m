%% Function to find trajectory of an agent
% Inputs: agent index, delta matrix that shows pairwise worst-case relaxations
% Outputs: state and input trajectory, relaxations per each predicate over the time

function [x_v,u_v,delta_v] = local_soln(i_agent,deltas)

global x0s x u H z delta p_stat p_dyn n_agents;

[A,B,u_limit] = agent_dynamics(i_agent); % Agent Dynamics (can be agent specific)

%% Define states and inputs as variables over the horizon
nx = length(A); % Number of states
nu = size(B,2); % Number of inputs

x = sdpvar(repmat(nx,1,H),repmat(1,1,H)); % State traj.
u = sdpvar(repmat(nu,1,H-1),repmat(1,1,H-1)); % Input policy

%% BUILD CONSTRAINTS

% STL constraints (Adjust per scenario!)
f_STL = cons_STL(i_agent);

% Initial state constraint
f_init = cons_init(x0s(:,i_agent));

% Dynamics constraint
f_dyn = cons_dyn(A,B);

% Available Input constraint
f_input = cons_input(u_limit);

f_delta=[];
for i=1:H
f_delta = [f_delta, delta{i}>=0]; % Time variant relaxations
end

% Append the constraints
f = [f_init,f_dyn,f_input,f_STL,z(1).('phi')==1,f_delta];

% Nominal Objective Function
Obj_sum = 0;
alpha1=10000;
alpha2=100;

for kk = 1:H-1
    Obj_sum=Obj_sum+abs(u{kk}(1))+abs(u{kk}(2))...
                    +alpha1*sum(delta{kk}(1:p_stat))...
                    +alpha2*sum(delta{kk}(p_stat+1:p_stat+p_dyn));
end
Obj_sum=Obj_sum+alpha1*sum(delta{H}(1:p_stat))...
               +alpha2*sum(delta{H}(p_stat+1:p_stat+p_dyn));

g = Obj_sum;

% Check if any inevitable relaxation between any two agents
m=0;
for j=1:n_agents
    if i_agent ~= j
        flag = 0; % bilateral relaxation flag
        m=m+1;
        for kk = 1:H 
            if deltas{kk}(i_agent,j)>0.01 && deltas{kk}(j,i_agent)>0.01
                flag = 1; % if ANY bilateral relaxation
            end
        end
        if flag==1 % if ANY bilateral relaxation, then penalize that couple more
            for kk = 1:H % collision avoidance with "that" agent penalized for whole duration
                g = g + 10*sum(delta{kk}(p_stat+1+(m-1)*8:p_stat+8+(m-1)*8));
            end
        end  
    end
end

opts = sdpsettings('verbose',0,'solver','gurobi');
soln = optimize(f,g,opts)
stage_cost = value(g)

% Get numerical position and input values
for i =1:H
    x_v(:,i) = value(x{i});
    delta_v(:,i) = value(delta{i}); % Get relaxation values of that agent
end
for i =1:H-1
    u_v(:,i) = value(u{i});
end

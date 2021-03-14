%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Author: Ali Tevfik Buyukkocak <buyuk012@umn.edu>  %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%     Distributed Planning of Multi-Agent Systems with Coupled %%%%%%%%%%
%%%               Temporal Logic Specifications                  %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note: The multiplications with 0.2875 are there to match numbers with 
% the experimental environment dimensions

clc;clear all;close all;clear global
addpath(genpath('.'))
global i_agent hrz H dt x0s rmin rmax boxes obstacles x_agents u_agents p_stat k n_agents
hrz = 40; % Mission horizon
dt = 1; % Time step
H = hrz/dt+1; % Total number of time steps
t = 0:dt:hrz; % Global time

%% Environment Description

BASE = [5.5 6.5 -2 2].*0.2875; % Base of agents for plotting purposes

% Regions of Interest
R_A = [-2 -4 6 4].*0.2875; % x's and y's: larger to smaller
R_B = [-5 -7 1 -1].*0.2875;
R_C = [-2 -4 -4 -6].*0.2875;

obstacles_plot = [0 1 0.75 4 ...
                  0 1 -4 -0.75].*0.2875; % x's and y's: smaller to larger

allboxes = [R_A R_B R_C obstacles_plot BASE]; % For drawing purposes

%% Agent parameters

x0s = [6 6 6; 0 1.5 -1.5].*0.2875; % Initial positions of agents
n_agents = size(x0s,2); % Number of agents in the mission

rad_coll = 0.7.*0.2875; % Radius of Coll. avodiance circle
rad_coh = 3*sqrt(2).*0.2875; % Radius of Coherence circle
rmin = rad_coll; % Half side length of outer fit box to circle (Safer than required) % MUST BE GREATER THAN U_LIMIT
rmax = rad_coh/sqrt(2); % Half side length of inner fit box to circle (More conservative)

% Real obstacles considering point mass agents
obstacles = [-0-rmin 1.*0.2875+rmin 0.75.*0.2875-rmin 4.*0.2875+rmin...
             -0-rmin 1.*0.2875+rmin -4.*0.2875-rmin -0.75.*0.2875+rmin]; % x's and y's: smaller to larger

% Agent-wise Regions of Interest (we have 3 agents here, all have same RoI)
boxes{1} = [R_A R_B R_C obstacles];
boxes{2} = [R_A R_B R_C obstacles];
boxes{3} = [R_A R_B R_C obstacles];

for kk=1:H
    deltas{kk} = zeros(n_agents,n_agents); % Pairwise relaxation matrix
end

k = 0; % Iteration number index

k_max=10; % maximum iteration number
while k<k_max
    k = k+1; 
for i_agent = 1:n_agents % Per each agent
    p_stat = size(boxes{i_agent},2); % Number of uncoupled predicates (depends on RoI)
    
    % Solve for each agent locally (first iteration has no coupling, just initial guesses)
    [x_v,u_v,delta_v] = local_soln(i_agent,deltas);
    
    for kk=1:H
        delta_v_i = delta_v(:,kk);
        delta_uncoupled = delta_v_i(1:size(boxes{i_agent},2)-size(obstacles,2));
        if max(delta_uncoupled)>0.09
            error('Mission is not feasible') % Even uncoupled predicates have to be relaxed
        end
        if k>1 % For iterations in which coupling occurs (2nd and later)
            m=0;
            for i =1:n_agents
                if i ~= i_agent
                    m=m+1;
                    delta_coupled = delta_v_i(p_stat+1+(m-1)*8:p_stat+8+(m-1)*8);% Total of 8 pairwise coupled predicates
                                                                                 % 4 coherence, 4 coll. avoid.
                    deltas{kk}(i,i_agent)=max(delta_coupled); % Find the worst pairwise relaxation
                end
            end
        end
            
    end   
    x_agents{i_agent} = x_v;
    u_agents{i_agent} = u_v;
end

% Check if any relaxation remains needed, if not break
if k>1 && any(any(cell2mat(deltas)>0.1))~=1
    fprintf(2,'Solved in %d iterations!\n',k)
    break
else
    fprintf(2,'Iteration %d failed!\n',k)
end
     
end

%% Visualization

figure (1)
hold all   
% Green boxes (Goal regions)
for i =1:(size(allboxes,2)-size(obstacles,2))/4-1 % -1 to exclude base
xbox=[allboxes(2+4*(i-1)) allboxes(2+4*(i-1)) allboxes(1+4*(i-1))...
      allboxes(1+4*(i-1)) allboxes(2+4*(i-1))];
ybox=[allboxes(4+4*(i-1)) allboxes(3+4*(i-1)) allboxes(3+4*(i-1))...
      allboxes(4+4*(i-1)) allboxes(4+4*(i-1))];

h1=fill(xbox,ybox,'g','LineWidth',0.001,'HandleVisibility','off');
set(h1,'facealpha',.15);
end

% Red boxes (Obstacles)
for i=1:size(obstacles_plot,2)/4
xobs=[obstacles_plot(2+4*(i-1)) obstacles_plot(2+4*(i-1)) obstacles_plot(1+4*(i-1))...
     obstacles_plot(1+4*(i-1)) obstacles_plot(2+4*(i-1))];
yobs=[obstacles_plot(4+4*(i-1)) obstacles_plot(3+4*(i-1)) obstacles_plot(3+4*(i-1))...
      obstacles_plot(4+4*(i-1)) obstacles_plot(4+4*(i-1))];

h2=fill(xobs,yobs,'r','LineWidth',0.001,'HandleVisibility','off');
set(h2,'facealpha',.40);
end

% Base
xbase=[BASE(2) BASE(2) BASE(1) BASE(1) BASE(2)];
ybase=[BASE(4) BASE(3) BASE(3) BASE(4) BASE(4)];
plot(xbase,ybase,'k','LineWidth',1,'HandleVisibility','off')

% Trajectories (Use low level MPC tracking or just discrete waypoints, comment out the other)
for i=1:n_agents
%[tcont,x_out,u_out] = low_level_mpc(t,x_agents{i},x0s(:,i),'makima'); plot(x_out(1,:).*0.2875,x_out(3,:).*0.2875,'LineStyle',':','Linewidth',3.5)
plot(x_agents{i}(1,:),x_agents{i}(2,:),'-*','Linewidth',1.5)
end

set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
xlabel('$x$ [m]','FontSize',18)
ylabel('$y$ [m]','FontSize',18)
ylim([-6.25 6.25].*0.2875)
xlim([-7.25 6.75].*0.2875)
set(gca,'FontSize',18)
legend('$Agent\ 1$','$Agent\ 2$','$Agent\ 3$','Location','Best')

%% Pairwise Distances
figure(2)
kk=0;
for i=1:n_agents
    for j=i+1:n_agents
        kk=kk+1;
        d{kk}=sqrt((x_agents{i}(1,:)-x_agents{j}(1,:)).^2 ...
                 +(x_agents{i}(2,:)-x_agents{j}(2,:)).^2);
             plot(t,d{kk},'LineWidth',1.5)
             hold all
    end
end

plot([t(1) t(end)],[0.2 0.2],'r',[t(1) t(end)],[1.2 1.2],'r','LineWidth',2.5) % Limits

set(groot,'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
ylabel('$d$ [m]','FontSize',18)
xlabel('$t$ [s]','FontSize',18)
ylim([-1 10].*0.2875)
set(gca,'FontSize',18)
hlegend=legend('$d_{1-2}$','$d_{1-3}$','$d_{2-3}$','Distance constraints','Location','Best');
hlegend.NumColumns=2;


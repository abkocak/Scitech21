function [A,B,u_limit] = agent_dynamics(i_agent)
global dt

% Function to transmit agent-specific dynamics

A = eye(2); % Simple dynamics are x_dot=u_x , y_dot=u_y
B = dt*eye(2); % OR x[k+1]=x[k]+dt*u_x[k] , y[k+1]=y[k]+dt*u_y[k] 

u_limit = 0.6*0.2875; % Magnitude of maximum input in both direction
% MUST BE LESS THAN R_MIN
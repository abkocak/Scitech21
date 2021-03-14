function [tcont,x_out,u_out] = low_level_mpc(tdisc,x_v,x0s,traj)
global dt hrz
Ts = dt/10;  % Sampling time [s]
Nt = hrz/Ts+1; % Horizon steps 0 to 60
tcont = linspace(0,(Nt-1)*Ts,Nt);
xcont=interpn(tdisc,x_v(1,:),tcont,traj);
ycont=interpn(tdisc,x_v(2,:),tcont,traj);

%% Choose horizon length, state/control constraints, predicted magnitude of w

N = 10; % Prediction horizon length (Time steps)

umax = [0.056;0.46*0.056;0.46*0.056;1/22*0.056];
umin = -umax;

xcont(Nt+1:Nt+N-2) = xcont(Nt); % For the horizon beyond the final time
ycont(Nt+1:Nt+N-2) = ycont(Nt); % For the horizon beyond the final time

%% Define constants
m = 0.027;          % Mass [kg]
g = 9.81;           % Gravity [m/s^2]
l = 0.046;          % Arm length [m]
I_x = 1.395e-5;     % MoI about x
I_y = 1.436e-5;     % MoI about y
I_z = 2.173e-5;     % MoI about z

%% Continous-Time State Space
Ac =[0  1  0  0  0  0  0  0  0  0  0  0;
     0  0  0  0  0  0  0  0  g  0  0  0;
     0  0  0  1  0  0  0  0  0  0  0  0;
     0  0  0  0  0  0 -g  0  0  0  0  0;
     0  0  0  0  0  1  0  0  0  0  0  0;
     0  0  0  0  0  0  0  0  0  0  0  0;
     0  0  0  0  0  0  0  1  0  0  0  0;
     0  0  0  0  0  0  0  0  0  0  0  0;
     0  0  0  0  0  0  0  0  0  1  0  0;
     0  0  0  0  0  0  0  0  0  0  0  0;
     0  0  0  0  0  0  0  0  0  0  0  1;
     0  0  0  0  0  0  0  0  0  0  0  0];
Bc =[0   0   0   0;
     0   0   0   0;
     0   0   0   0;
     0   0   0   0;
     0   0   0   0;
   -1/m  0   0   0;
     0   0   0   0;
     0 l/I_x 0   0;
     0   0   0   0;
     0   0 l/I_y 0;
     0   0   0   0;
     0   0   0 1/I_z];

Cc = [1 0 0 0 0 0 0 0 0 0 0 0;
      0 0 1 0 0 0 0 0 0 0 0 0;
      0 0 0 0 1 0 0 0 0 0 0 0;
      0 0 0 0 0 0 1 0 0 0 0 0;
      0 0 0 0 0 0 0 0 1 0 0 0;
      0 0 0 0 0 0 0 0 0 0 1 0];
Dc = 0;
sysc = ss(Ac,Bc,Cc,Dc); % Continuous-time state space

nx = size(Ac,1);
nu = size(Bc,2);
ny = size(Cc,1);

%% Discrete-Time State Space
sysd = c2d(sysc,Ts,'zoh');          % Discrete-time state space with ZOH
[Ad, Bd, Cd, Dd] = ssdata(sysd);    % Discrete state space Matrices

%% Choose weights for cost function
Q = blkdiag(10000,1,10000,1,10000,1,1000,10,1000,10,1000,10);
R = blkdiag(1/0.056,1/(0.46*0.056),1/(0.46*0.056),1/(1/22*0.056));
% Solve for infinite-horizon D-T LQR solution
[K_LQ,P_LQ,E_LQ] = dlqr(Ad,Bd,Q,R);


%% Construct Matrices for MPC computation

S2 = [];
M = [];
Qbar = [];
Rbar = [];
Umin = [];
Umax = [];
for lv1 = 1:N
    lv1;
    S2_i = [];
    
    for lv2 = 1:lv1
        S2_i = [S2_i Ad^(lv1-lv2)*Bd];
    end
    S2 = [S2; S2_i zeros(nx,nu*(N-lv1))];
    M = [M;Ad^lv1];
    
    if lv1 == N
        Qbar = blkdiag(Qbar,P_LQ);
    else 
        Qbar = blkdiag(Qbar,Q);
    end
    Rbar = blkdiag(Rbar,R);
    
    Umin = [Umin;umin];
    Umax = [Umax;umax]; 
end

% Constraints on input
G_input = [eye(N*nu);-eye(N*nu)];
Z_input = [Umax;-Umin];
T_input = zeros(2*N*nu,nx);
L_input = zeros(2*N*nu,nu);

% Combined Constraints
G = [G_input];
Z = [Z_input];
T = [T_input];
L = [L_input];

%% Perform Simulation

xt0 = [x0s(1); 0; x0s(2); zeros(9,1)]; % initial condition
x_out = xt0;
x_LQ = xt0;
u_out = [];
u_LQ = [];

% Loop through simulation 2 times (first with LQ-MPC, second with LQR)
for lv1 = 1:1
    xk = xt0;
    %u_prev = zeros(4,1); % Initiate u(k-1) with zero
  
    for lv2 = 1:Nt-1
        W = [];    
        for i = 1:N
            W = [W; xcont(lv2+(i-1));0;ycont(lv2+(i-1));zeros(9,1)];
        end
        
        %State at time step
        xt = xk;
        
        % Calcuated control input
        if lv1 == 1 % LQ-MPC
            H = S2'*Qbar*S2 + Rbar;
            q = S2'*Qbar*(M*xt-W);

            Ztilde = Z + T*xt; %+ L*u_prev;
            options = optimoptions('quadprog','Display','off');

            [U,fval,exitflag] = quadprog(2*H,2*q,G,Ztilde);


            uk = U(1:nu);
        else % LQR
            uk = -K_LQ*(xt-W(1:nx,:)); % Assumed r_dot = 0;
                                       % Then e_dot = A*e + B*u
        end

    % Propogate state
    xk1 = Ad*xk + Bd*uk;
    
    % Save data
    if lv1 == 1
        u_out = [u_out uk];
        x_out = [x_out xk1];
    else
        u_LQ = [u_LQ uk];
        x_LQ = [x_LQ xk1];
    end

    xk = xk1; % Update x(k)
    end

end

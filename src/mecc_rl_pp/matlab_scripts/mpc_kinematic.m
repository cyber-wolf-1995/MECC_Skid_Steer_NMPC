function Umpc = mpc_kinematic(waypoints, x0)
addpath("/home/ssathe/casadi-linux-matlabR2014b-v3.5.5")
import casadi.*
%% Vehicle Parameters
W = 0.555;            % Wheel Track in m
r = 0.165;            % Wheel Radius in m
chi = 1;              % Empirical Parameter related to ICR 
v_max = 1;            % Max Longitudinal speed in m/s
vmin = -v_max;        % Minimum Longitudinal speed in m/s
w_max = v_max/r;      % Maximum Angular Speed rad/s
w_min = -w_max;       % Minimum Angular Speed rad/s
%% Simulation/ Controller Parameters
ts = 1/10;               % Sample time
N = 10;               % Number of Iteration
%% Declare Variables for Optimization
x = SX.sym('x');
y = SX.sym('y');
theta = SX.sym('theta');
states = [x; y; theta];
nx = length(states);
wl = SX.sym('wl');
wr = SX.sym('wr');
controls = [wl; wr];
nu = length(controls);

rhs = [(wl+wr)*(r/2)*cos(theta); (wl+wr)*(r/2)*sin(theta); (-r/W/chi)*wl+(r/W/chi)*wr];


f = Function('f', {states, controls}, {rhs});


U = SX.sym('U', nu, N);    % Control Inputs
p = SX.sym('P', nx + N*(nx-1));  % Parameters

X = SX.sym('X', nx, N+1);  % States

%% Formulate Problem

obj = 0; % Objective function
g = [];  % constraints vector

X_init  = X(:,1); % initial state
g = [g; X_init - p(1:3)]; % initial condition constraints

wx = 30;
wdudt = 0.1;
wu =0.1;
uref = [w_max; w_max];
%uref = [0; 0];
Q = wx*eye(2,2);
P = wdudt*eye(2,2);
R = wu*eye(2,2);
for k=1:N
    state = X(:,k);
    control = U(:,k);
    if k>1
        u1 = U(:,k-1);
    else
        u1 = zeros(nu,1);
    end
    u2 = control;
    obj = obj + (state(1:2) - p(2*k+2:2*k+3))'*Q*(state(1:2) - p(2*k+2:2*k+3)) + ((u2-u1)/ts)'*P*((u2-u1)/ts) + (abs(u2) - uref)'*R*(abs(u2) -uref);
    %obj = obj + (state(1:2) - p(2*k+2:2*k+3))'*Q*(state(1:2) - p(2*k+2:2*k+3)) + ((u2-u1)/ts)'*P*((u2-u1)/ts) + (u2 - uref)'*R*(u2 -uref);
    X_dot = f(state, control);
    X_next = state + ts*X_dot;
    X_next_euler = X(:,k+1);
    g = [g;X_next-X_next_euler]; % compute constraints
end

% make the decision variables one column vector
OPT_variables = [reshape(X,nx*(N+1),1);reshape(U,nu*N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', p);

opts = struct;
opts.ipopt.max_iter = 10;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);


args = struct;

args.lbg(1:3*(N+1)) = 0;  % -1e-20  % Equality constraints
args.ubg(1:3*(N+1)) = 0;  % 1e-20   % Equality constraints

args.lbx(1:3:3*(N+1),1) = -6; %state x lower bound % new - adapt the bound
args.ubx(1:3:3*(N+1),1) = 5; %state x upper bound  % new - adapt the bound
args.lbx(2:3:3*(N+1),1) = -5.5; %state y lower bound
args.ubx(2:3:3*(N+1),1) = 1; %state y upper bound
args.lbx(3:3:3*(N+1),1) = -inf; %state theta lower bound
args.ubx(3:3:3*(N+1),1) = inf; %state theta upper bound

% input constraints
args.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = w_min;     % w_left lower bound
args.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = w_max;     % w_left upper bound
args.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = w_min;     % w_right lower bound
args.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = w_max;     % w_right upper bound
 
%% Start Simulation
t0 = 0;
%x0 = [0 ; 0 ; 0.0];    % initial condition.
%x0 = [0 ; 0 ; 0.0]; % Reference posture.

xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,2);        % two control inputs 
X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables

sim_tim = 0.2; % Maximum simulation time

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];

% the main simulaton loop... it works as long as the error is greater
% than 10^-6 and the number of mpc steps is less than its maximum
% value.
%main_loop = tic;
while(mpciter < sim_tim / ts) % new - condition for ending the loop
    current_time = mpciter*ts;  %new - get the current time
    % args.p   = [x0;xs]; % set the values of the parameters vector
    %----------------------------------------------------------------------
    args.p(1:3) = x0; % initial condition of the robot posture
    for k = 1:N %new - set the reference to track
        t_predict = current_time + (k-1)*ts; % predicted time instant
        x_ref = waypoints(k,1); y_ref = waypoints(k,2);
        args.p(2*k+2:2*k+3) = [x_ref, y_ref];

    end
    %----------------------------------------------------------------------    
    % initial value of the optimization variables
    args.x0  = [reshape(X0',3*(N+1),1);reshape(u0',2*N,1)];
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)'; % get controls only from the solution
    xx1(:,1:3,mpciter+1)= reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    % Apply the control and shift the solution
    [t0, x0, u0] = shift(ts, t0, x0, u,f);
    xx(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    %mpciter
    mpciter = mpciter + 1;
end
%main_loop_time = toc(main_loop);
%average_mpc_time = main_loop_time/(mpciter+1);

vx = u(:,1)*r/2 + u(:,2)*r/2;
wz = (r/(W*chi))*(u(:,2) - u(:,1));
Umpc = [vx'; wz'];
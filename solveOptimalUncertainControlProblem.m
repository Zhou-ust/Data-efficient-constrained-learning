function [ x, u ] = solveOptimalUncertainControlProblem(plant, t, x_t, u_t, N, Vfun, XX, UU, UBounds, XBounds)
% Clear Yalmip Variables
yalmip('clear')

n = size(XX,1);
m = size(UU,1);

x = sdpvar(n*ones(1,N+1),ones(1,N+1));
u = sdpvar(m*ones(1,N),ones(1,N));
lambda = sdpvar(length(Vfun), 1);

% Initial Condition
Constraints = [x_t == x{1}];

% System Dynamics
At = [1.6070 1 0;
    -0.6086  0  0;
    1.6070 1  1];
Bt = [1.2390;-0.9282;1.2390];

for i = 1:N
    [Uset, Xset] = constraints(u_t, x_t, UBounds, XBounds);
    Hx  = Xset.A;  bx  = Xset.b;
    Hu  = Uset.A;  bu  = Uset.b;
    
    Dt = [0;0; 1*(reference(i) - reference(i+1))];
    Constraints = [Constraints;
        x{i+1} == At*x{i} + Bt*u{i} + Dt;
        % x{i+1} == plant( t + i,x{i},u{i});
        Hx * x{i} <= bx;
        Hu * u{i} <= bu;];
end

for i = 1:length(Vfun)
    Constraints = [Constraints;
        lambda(i) >= 0;];
end

Constraints = [Constraints;
    x{N+1} == XX*lambda;                % Terminal point in the convex hull
    ones(1,length(Vfun))*lambda == 1];  % Must be convex combination --> sum to 1

% Running Cost
Cost = 0;
for i = 1:N
    Cost = Cost + runningcost( t + i, x{i}, u{i});
end

% New cost function
Cost = Cost + 1*Vfun*lambda;


options = sdpsettings('solver','gurobi','verbose',0);
Problem = optimize(Constraints, Cost, options);

Objective = double(Cost);
cost = value(Cost);
% yalmip('clear')

end


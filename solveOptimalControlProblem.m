function [ x, u ] = solveOptimalControlProblem(plant, t, x_t, u_t, N, Vfun, XX, UU, UBounds, XBounds)

yalmip('clear')

n = size(XX,1);
m = size(UU,1);

x = sdpvar(n*ones(1,N+1),ones(1,N+1));
u = sdpvar(m*ones(1,N),ones(1,N));
lambda = sdpvar(length(Vfun), 1);

Constraints = [x_t == x{1}];

for i = 1:N
    [Uset, Xset] = constraints(u_t, x_t, UBounds, XBounds);
    Hx  = Xset.A;  bx  = Xset.b;
    Hu  = Uset.A;  bu  = Uset.b;
    
    Constraints = [Constraints;
        x{i+1} == plant( t + i,x{i},u{i});
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
Cost = Cost + Vfun*lambda;


options = sdpsettings('solver','gurobi','verbose',0);
Problem = optimize(Constraints, Cost, options);

Objective = double(Cost);
cost = value(Cost);
% yalmip('clear')

end


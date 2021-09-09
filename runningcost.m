function cost = runningcost(t, x, u)

cost = 0.1.*(x(1))^2 + 0.1.*(x(2))^2 + 1.*(x(3))^2 + 0.1*u(1)^2;
end

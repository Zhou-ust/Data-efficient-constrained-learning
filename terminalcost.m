function cost = terminalcost(t, x)

if size(x,2) == 1
    x = x;
else
    x = x';
end

cost =  0.1*(x(1))^2 + 0.1.*(x(2))^2 + 1.*(x(3))^2;
end

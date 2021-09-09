
function [Uset,Xset] = constraints(u1, x1, UBounds, XBounds)

% bounds = [umax umin deltaumax deltaumin xmax xmin deltaxmax deltaxmin];
umax = UBounds(1);
umin = UBounds(2);
deltaumax = UBounds(3);
deltaumin = UBounds(4);

xmax = XBounds(:,1);
xmin = XBounds(:,2);
deltaxmax = XBounds(:,3);
deltaxmin = XBounds(:,4);



% Define the input constraint set Uset
m = size(u1,1);

Gammau = [eye(m); -eye(m)];
Lambdau = [min(deltaumax,umax - u1); -max(deltaumin,umin - u1 )];

Uset = Polyhedron(Gammau,Lambdau );




% Define the state constraint set Xset
n = max(size(x1));

Gammax = [eye(n); -eye(n)];
Lambdax = [min(deltaxmax,xmax - x1); -max(deltaxmin,xmin - x1 )];

Xset = Polyhedron(Gammax,Lambdax );

end

function y = plant(t, x, u)

At = [1.6070 1 0;
    -0.6086  0  0;
    1.6070 1  1];
Bt = [1.2390;-0.9282;1.2390];

xx = [x(1), x(2)];
n = norm(xx,2);

% y = A*x +  B*u ;

y = At*x +  Bt*u + 1.0*[0.08*sin(10*t)*x(1);0.08*cos(15*t)*x(1);0.08*sin(10*t)*x(1)] + ...
    [0;0; 1*(reference(t) - reference(t+1))];

end


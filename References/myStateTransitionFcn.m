% Copyright 2018 The MathWorks, Inc. 

function x = myStateTransitionFcn(x,u)
% Sample time [s]
dt = 0.01; 

% Using Euler discretization, next states
% can be calculated given the current
% states and input
g=9.81;
L=1;
dxdt=[x(2); -sqrt(g/L)*sin(x(1))];
x = x + dxdt*dt;
end


% Copyright 2018 The MathWorks, Inc. 

function y = myMeasurementFcn(x)
% x1: Angular position (theta) 
L=1;
y = L*sin(x(1)); 
end
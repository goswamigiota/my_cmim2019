% Use Newton-Raphson?s method to solve systems of nonlinear algebraic equations.
function [x, iteration_counter] = NR_method(F, J, x, eps, maxiter,phi)
% Solve nonlinear system F=0 by Newton-Raphson?s method.
% J is the Jacobian of F. Both F and J must be functions of x.
% At input, x holds the start value. The iteration continues
% until ||F|| < eps.
F_value = F(x,phi);
F_norm = norm(F_value); % L2 norm of vector
iteration_counter = 0;
while F_norm > eps && iteration_counter < maxiter
    delta = J(x)\-F_value;
    x = x + delta;
    F_value = F(x,phi);
    F_norm = norm(F_value);
    iteration_counter = iteration_counter + 1;
end
% Here, either a solution is found, or too many iterations
if F_norm > eps
    iteration_counter = -1;
end
end
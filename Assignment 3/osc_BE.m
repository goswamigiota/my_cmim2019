%  S. Linge and H. P. Langtangen, Programming for Computations - MATLAB/Octave: A
%  Gentle Introduction to Numerical Simulations with MATLAB/Octave. 2016.
%  Exercise 4.14: Use a Backward Euler scheme for oscillations (b) (Page 149)
%  Solved by Giota Goswami (0524987)

%  Program to implement the found formulas for un and vn.

omega = 2;
P = 2*pi/omega;
% dt = P/20;
dt = P/2000;
T = 3*P;
N_t = floor(round(T/dt));
t = linspace(0, N_t*dt, N_t+1);

u = zeros(N_t+1, 1);
v = zeros(N_t+1, 1);

% Initial condition
X_0 = 2;
u(1) = X_0;
v(1) = 0;

% Step equations forward in time
for n = 2:N_t+1
    u(n) = (1.0/(1+(dt*omega)^2)) * (dt*v(n-1) + u(n-1));
    v(n) = (1.0/(1+(dt*omega)^2)) * (-dt*omega^2*u(n-1) + v(n-1));
end

plot(t, u, 'b-', t, X_0*cos(omega*t), 'r--');
legend('numerical', 'exact', 'Location', 'northwest');
xlabel('t');
print('tmp', '-dpdf');  print('tmp', '-dpng');

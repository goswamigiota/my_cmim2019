%  S. Linge and H. P. Langtangen, Programming for Computations - MATLAB/Octave: A
%  Gentle Introduction to Numerical Simulations with MATLAB/Octave. 2016.
%  Exercise 4.10: Compute the energy in oscillations (b) (Page 147)
%  Solved by Giota Goswami (0524987)

%  Program to plot the sum of potential and kinetic energy in oscillations
%  using Euler-Cromer method.

omega = 2;
P = 2*pi/omega;
dt = P/20;
T = 40*P;
N_t = floor(round(T/dt));
t = linspace(0, N_t*dt, N_t+1);

u = zeros(N_t+1, 1);
v = zeros(N_t+1, 1);

% Initial condition
X_0 = 2;
u(1) = X_0;
v(1) = 0;

% Step equations forward in time
for n = 1:N_t
    u(n+1) = u(n) + dt*v(n+1);
    v(n+1) = v(n) - dt*omega^2*u(n);
end

[U, K] = osc_energy(u, v, omega);

plot(t, U+K, 'b-');
xlabel('t');
ylabel('U+K');

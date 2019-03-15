%  S. Linge and H. P. Langtangen, Programming for Computations - MATLAB/Octave: A
%  Gentle Introduction to Numerical Simulations with MATLAB/Octave. 2016.
%  ode_FE function taken from page 109 of the book

%  General function to solve any vector ODEs.

function [u, t] = ode_FE(f, U_0, dt, T) 
    N_t = floor(T/dt); 
    u = zeros(N_t+1, length(U_0)); 
    t = linspace(0, N_t*dt, length(u)); 
    u(1,:) = U_0;      % Initial values 
    t(1) = 0; 
    for n = 1:N_t 
        u(n+1,:)  = u(n,:) + dt*f(u(n,:), t(n)); 
    end 
 end 

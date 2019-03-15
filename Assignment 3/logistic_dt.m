%  S. Linge and H. P. Langtangen, Programming for Computations - MATLAB/Octave: A
%  Gentle Introduction to Numerical Simulations with MATLAB/Octave. 2016.
%  Exercise 4.4: Find an appropriate time step; logistic model (Page 145)
%  Solved by Giota Goswami (0524987)

%  Program to refine the numerical solution of the logistic equation
%  by helving time steps repeatedly.

clear all;
close all;
clc;

f = @(u, t) 0.1*(1 - u/500)*u;
U_0 = 100;
dt = 50; T = 100;
[u, t] = ode_FE(f, U_0, dt, T);

for k=1:100
    dt_new=2^(-k)*dt;
    [u_new, t_new] = ode_FE(f, U_0, dt_new, T);
    figure
    plot(t, u, 'b-', t_new, u_new, 'r--');
    xlabel('t'); ylabel('N(t)');
    fprintf('Timestep was: %g\n', dt_new);
    prompt = input('Continue with finer dt (y/n)? ', 's');
    if prompt=='n'
        break;
    else
        u = u_new;
        t = t_new;
    end
end
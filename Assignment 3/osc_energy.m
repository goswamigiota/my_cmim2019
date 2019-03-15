%  S. Linge and H. P. Langtangen, Programming for Computations - MATLAB/Octave: A
%  Gentle Introduction to Numerical Simulations with MATLAB/Octave. 2016.
%  Exercise 4.10: Compute the energy in oscillations (a) (Page 147)
%  Solved by Giota Goswami (0524987)

%  Function for returning the potential and kinetic energy of an oscillating system.

function [U, K] = osc_energy(u, v, omega)
    U = 0.5*omega.^2*u.^2;      % Potential energy
    K = 0.5*v.^2;               % Kinetic energy
end
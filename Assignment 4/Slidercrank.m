%  Computational Methods in Mechanics 2019
%  Assignment 4: Kinematic analysis of a slider crank mechanism
%  Parviz E. Nikravesh-Computer-Aided Analysis of Mechanical 
%  Systems-Prentice Hall (1988) Page 8
%  Solved by Giota Goswami (0524987)

%  Program to solve position, velocity and acceleration of a simple
%  slider-crank mechanism using Newton-Raphson's method.

clear all;
close all;
clc;

a=0.1;
b=0.2;
phi=30*pi/180;

f=
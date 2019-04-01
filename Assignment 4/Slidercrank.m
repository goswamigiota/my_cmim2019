%  Computational Methods in Mechanics 2019
%  Assignment 4: Kinematic analysis of a slider crank mechanism
%  Parviz E. Nikravesh-Computer-Aided Analysis of Mechanical 
%  Systems (1988) Page 8
%  Solved by Giota Goswami (0524987)

%  Program to solve position, velocity and acceleration of a simple
%  slider-crank mechanism using Newton-Raphson's method.

clear all;
close all;
clc;

a=0.1;                          % Length OA
b=0.2;                          % Length AB
om=1;                           % Angular velocity omega
t = 0:0.01:10;                  % Time interval
tol = 1e-6;                     % Tolerance for Newton-Raphson
maxii = 1000;                   % Maximum iteration for Newton-Raphson

f=@(x,phi) [a*cos(phi)+b*cos(x(1)) - x(2);      % Function f
            a*sin(phi)-b*sin(x(1))];
J=@(x) [-b*sin(x(1)), -1;                       % Jacobian of f
        -b*cos(x(1)), 0];
dfdt=@(phi) [-a*sin(phi)*om;                    % Time derivative of f
              a*cos(phi)*om];
G=@(x,x_dot,phi) [a*cos(phi)*om^2+b*cos(x(1))*x_dot(1)^2;
                  a*sin(phi)*om^2-b*sin(x(1))*x_dot(1)^2];

for i = 1:length(t)
    
    xini=[asin(1/4);                            % Initial values
           0.28];
    phi=om*t(i)+pi/6;
     
    [x,it_count]=NR_method(f,J,xini,tol,maxii,phi);
    
    x_dot=J(x)\-dfdt(phi);                      % Velocity
    x_2dot=J(x)\G(x,x_dot,phi);                 % Acceleration 
    x_save(:,i)=x;                              % Position    
    x_dot_save(:,i)=x_dot;
    x_2dot_save(:,i)=x_2dot; 

end

% Theta plot
figure(1);
plot(t, x_save(1,:)*180/pi,'LineWidth',2)
hold on
set(gca,'FontSize',12, 'FontName', 'Times New Roman');
ylabel ('Angle \theta ({\circ})','FontSize',12, 'FontName', 'Times New Roman');
xlabel ('Time t (s)','FontSize',12, 'FontName', 'Times New Roman');
hold off

% Position plot
figure(2);
plot(t, x_save(2,:),'LineWidth',2)
hold on
set(gca,'FontSize',12, 'FontName', 'Times New Roman');
ylabel ('Displacement d (m)','FontSize',12, 'FontName', 'Times New Roman');
xlabel ('Time t (s)','FontSize',12, 'FontName', 'Times New Roman');
hold off

% Angular velocity plot
figure(3);
plot(t, x_dot_save(1,:),'LineWidth',2)
hold on
set(gca,'FontSize',12, 'FontName', 'Times New Roman');
ylabel ('Angular velocity $\dot{\theta}$ (rad/s)','FontSize',12, 'FontName', 'Times New Roman', 'Interpreter','latex');
xlabel ('Time t (s)','FontSize',12, 'FontName', 'Times New Roman');
hold off

% Velocity plot
figure(4);
plot(t, x_dot_save(2,:),'LineWidth',2)
hold on
set(gca,'FontSize',12, 'FontName', 'Times New Roman');
ylabel ('Velocity $\dot{d}$ (m/s)','FontSize',12, 'FontName', 'Times New Roman', 'Interpreter','latex');
xlabel ('Time t (s)','FontSize',12, 'FontName', 'Times New Roman');
hold off

% Angular acceleration plot
figure(5);
plot(t, x_2dot_save(1,:),'LineWidth',2)
hold on
set(gca,'FontSize',12, 'FontName', 'Times New Roman');
ylabel ('Angular acceleration $\ddot{\theta}$ (rad/s$^2$)','FontSize',12, 'FontName', 'Times New Roman', 'Interpreter','latex');
xlabel ('Time t (s)','FontSize',12, 'FontName', 'Times New Roman');
hold off

% Acceleration plot
figure(6);
plot(t, x_2dot_save(2,:),'LineWidth',2)
hold on
set(gca,'FontSize',12, 'FontName', 'Times New Roman');
ylabel ('Acceleration $\ddot{d}$ (m/s$^2$)','FontSize',12, 'FontName', 'Times New Roman', 'Interpreter','latex');
xlabel ('Time t (s)','FontSize',12, 'FontName', 'Times New Roman');
hold off

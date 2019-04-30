clear all;
close all;
clc;

% Slider crank kinematic analysis
%% Coordinates
% ground
q1 = [0; 0; 0];
% crank
q2 = [-0.1 * cosd(30)
    0.1 * sind(30)
    -deg2rad(30)];
% link
h_B = 0.2 * sind(30); % y coordinate of point B
phi_l = asin(h_B / 0.5); % link's angle
q3 = [-0.2 * cosd(30) - 0.3 * cos(phi_l)
    h_B - 0.3 * sin(phi_l)
    phi_l];
% slider
q4 = [-0.2 * cosd(30) - 0.5 * cos(phi_l)
    0
    0];

q_0 = [q1; q2; q3; q4]; % initial coordinates

%% We need two constraint types (geometric ones)
% - revolute
% - simple constraints

%% Revolute joints
% 1 connects ground and crank
revolute(1).i = 1;
revolute(1).j = 2;
revolute(1).s_i = [0; 0];
revolute(1).s_j = [0.1; 0];

% 2 connects crank and link
revolute(2).i = 2;
revolute(2).j = 3;
revolute(2).s_i = [-0.1; 0];
revolute(2).s_j = [0.3; 0];

% 3 connects link and slider
revolute(3).i = 3;
revolute(3).j = 4;
revolute(3).s_i = [-0.2; 0];
revolute(3).s_j = [0; 0];

% % Check revolute joint constraints
% r = revolute(3);
% C_r_i = revolute_joint(r.i, r.j, r.s_i, r.s_j, q_0)

%% Simple constraints

% Three simple joints to fix the ground origin
simple(1).i = 1;
simple(1).k = 1;
simple(1).c_k = 0;

simple(2).i = 1;
simple(2).k = 2;
simple(2).c_k = 0;

simple(3).i = 1;
simple(3).k = 3;
simple(3).c_k = 0;

% slider - use simple joints instead of translational
simple(4).i = 4;
simple(4).k = 2;
simple(4).c_k = 0;

simple(5).i = 4;
simple(5).k = 3;
simple(5).c_k = 0;

% % check simple constraints
% for s = simple
%     C_s_i = simple_joint(s.i, s.k, s.c_k, q_0)
% end

%% Add some driving constraints
driving.i = 2;
driving.k = 3;
driving.d_k = @(t) -pi/6 - 1.2 * t;
driving.d_k_t = @(t) -1.2;
driving.d_k_tt = @(t) 0;

% % Verify
% d = driving(1);
% C_d_i = driving_joint(d.i, d.k, d.d_k, 0, q_0)

% %% Verify constraint function
% clc
% C = constraint(revolute, simple, driving, 0, q_0)

% %% Solve constraint equation using fsolve
% C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
% [T, Q] = position_fsolve(C_fun, 1, q_0, 0.1);
% 
% %% Some verification plots
% plot(Q(:, 4), Q(:, 5), ...
%     Q(:, 7), Q(:, 8), ...
%     Q(:, 10), Q(:, 11), ...
%     0, 0, '*', 'LineWidth', 2);
% axis equal
% 
% %% Jacobian of our constraints
% Cq = constraint_dq(revolute, simple, driving, 0, q_0)
% 
% %% Solve constraint equation using NR
% C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
% Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
% [T, Q] = position_NR(C_fun, Cq_fun, 1, q_0, 0.1);
% 
% %% Some verification plots
% plot(Q(:, 4), Q(:, 5), ...
%     Q(:, 7), Q(:, 8), ...
%     Q(:, 10), Q(:, 11), ...
%     0, 0, '*', 'LineWidth', 2);
% axis equal
% 
% %% Verify Ct
% Ct = constraint_dt(revolute, simple, driving, 0, q_0);
% 
% %% Solve constraint equation using NR for position and velocity
% C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
% Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
% Ct_fun = @(t, q) constraint_dt(revolute, simple, driving, t, q);
% [T, Q, QP] = pos_vel_NR(C_fun, Cq_fun, Ct_fun, 1, q_0, 0.1);
% 
% %% Some verification plots
% plot(Q(:, 4), Q(:, 5), ...
%     Q(:, 7), Q(:, 8), ...
%     Q(:, 10), Q(:, 11), ...
%     0, 0, '*', 'LineWidth', 2);
% axis equal
% 
% 
% %% Some verification plots
% plot(QP(:, 4), QP(:, 5), ...
%     QP(:, 7), QP(:, 8), ...
%     QP(:, 10), QP(:, 11), ...
%     0, 0, '*', 'LineWidth', 2);
% axis equal

%% Solve constraint equation using NR for position, velocity and acceleration
C_fun = @(t, q) constraint(revolute, simple, driving, t, q);
Cq_fun = @(t, q) constraint_dq(revolute, simple, driving, t, q);
Ct_fun = @(t, q) constraint_dt(revolute, simple, driving, t, q);
g_fun= @(t, q, dq) g(revolute, simple, driving, t, q, dq);
[T, Q, QP, QPP] = pos_vel_acc_NR(C_fun, Cq_fun, Ct_fun, g_fun, 1, q_0, 0.1);

%% Some verification plots
figure
plot(Q(:, 4), Q(:, 5), ...
    Q(:, 7), Q(:, 8), ...
    Q(:, 10), Q(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal


%% Some verification plots
figure
plot(QP(:, 4), QP(:, 5), ...
    QP(:, 7), QP(:, 8), ...
    QP(:, 10), QP(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal

%% Some verification plots
figure
plot(QPP(:, 4), QPP(:, 5), ...
    QPP(:, 7), QPP(:, 8), ...
    QPP(:, 10), QPP(:, 11), ...
    0, 0, '*', 'LineWidth', 2);
axis equal

%% Dynamic Analysis
%  Define bodies
body(1).m = 0; % ground mass equals to 0 kg
body(1).l = 0; 
body(1).Ic = body(1).m * body(1).l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(1).q = [0; 0; 0];

body(2).m = 2; % crank mass equals to 2 kg
body(2).l = 0.2; 
body(2).Ic = body(2).m * body(2).l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(2).q = [-0.1 * cosd(30)
    0.1 * sind(30)
    -deg2rad(30)];

body(3).m = 2; % link mass equals to 2 kg
body(3).l = 0.5; 
body(3).Ic = body(3).m * body(3).l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(3).q = [-0.2 * cosd(30) - 0.3 * cos(phi_l)
    h_B - 0.3 * sin(phi_l)
    phi_l];

body(4).m = 2; % slider mass equals to 2 kg
body(4).l = 0; 
body(4).Ic = body(4).m * body(4).l^2 / 12; % mass moment of inertia along center of mass in kgm2
body(4).q = [-0.2 * cosd(30) - 0.5 * cos(phi_l)
    0
    0];

param.grav = [0; -9.81]; % gravitational acceleration

%% Get mass matrix

param.M = mass_matrix(body);
q0 = [q_0;
    zeros(length(q_0), 1)];
tspan = 0:0.05:10;

sforce.f = [1; 0];
sforce.i = 1;
sforce.u_i = [0; 1];

param.bodies = body;
% g_fun= @(t, q, dq) g_baum(revolute, simple, driving, t, q, dq);

param.C_fun = @(t, q) constraint_dynamic(revolute, simple, t, q);
param.Cq_fun = @(t, q) constraint_dq_dynamic(revolute, simple, t, q);
param.g_fun= @(t, q, dq) g_dynamic(revolute, simple, t, q, dq);

%% Time to integrate it
% Note that M is constant, but F, in general, no
% We can use one of the following:
%   ode45 from Matlab
%   Euler-Cromer as introduced some time ago
%   Lets try Euler-Cromer
% acc_f = @(t, q, qp) system_accelerations(t, q, qp, M, sforce, grav, body);
options = odeset('Stats', 'on','RelTol',1e-6);
tic
[t, y] = ode45(@eom, tspan, q0, options, param);
toc
sol = y';

% Animation of Dynamic Analysis
figure
for iii = 1:length(tspan)
    clf
    hold on
    
    r1_J1 = [sol(4,iii);sol(5,iii)] + [cos(sol(6,iii)) -sin(sol(6,iii)); sin(sol(6,iii)) cos(sol(6,iii))]*[body(2).l/2;0];
    r1_J2 = [sol(4,iii);sol(5,iii)] + [cos(sol(6,iii)) -sin(sol(6,iii)); sin(sol(6,iii)) cos(sol(6,iii))]*[-body(2).l/2;0];
    r2_J2 = [sol(7,iii);sol(8,iii)] + [cos(sol(9,iii)) -sin(sol(9,iii)); sin(sol(9,iii)) cos(sol(9,iii))]*[body(3).l*3/5;0];
    r2_J3 = [sol(7,iii);sol(8,iii)] + [cos(sol(9,iii)) -sin(sol(9,iii)); sin(sol(9,iii)) cos(sol(9,iii))]*[-body(3).l*2/5;0];
    r3_J3 = [sol(10,iii);sol(11,iii)] + [cos(sol(12,iii)) -sin(sol(12,iii)); sin(sol(12,iii)) cos(sol(12,iii))]*[0;0];

    plot([r1_J1(1), r1_J2(1)], [r1_J1(2), r1_J2(2)], 'LineWidth',2)
    plot([r2_J2(1), r2_J3(1)], [r2_J2(2), r2_J3(2)], 'LineWidth',2)
    plot(r3_J3(1), r3_J3(2),'*', 'LineWidth',4)
    plot(0,0, '*', 'LineWidth', 4)
    
    axis([- 0.75 0.25 -0.5 0.5])

    xlabel('Position, ${q_x}$ (m)','FontSize',12, 'FontName', 'Times New Roman','interpreter','latex');
    ylabel('Position, ${q_y}$ (m)','FontSize',12, 'FontName', 'Times New Roman','interpreter','latex');
    legend({'Crank','Connection rod','Slider','Origin'},'Location','northeast', 'FontSize',12, 'FontName', 'Times New Roman')
    
    title('Dynamic Analysis of Slider Crank Mechanism','FontSize',12, 'FontName', 'Times New Roman')
    set(gca,'FontSize',12, 'FontName', 'Times New Roman');
    
    pause(0.05)
    drawnow
    
end
%addpath(genpath('C:\Users\mitadm\Downloads\gloptipoly3\gloptipoly3'))
%addpath(genpath('C:\Users\mitadm\Downloads\YALMIP-master\YALMIP-master'))
%addpath(genpath('C:\Users\mitadm\Downloads\Mosek\9.1'))

clear all;
close all;

initial_loc = [-0.8, -0.6, 0];
psi_primitives = [0, 30, -30, 60, -60];%15, 15, -30, 30, -45, 45, -60, 60]; % degrees
obstacles = [[0.1, -0.15, 0.45]; [0.1, 0.7, 0.1]; [0.1, -0.5, -0.6]]; % [r xc yc]
xo = 0.5; yo = 0.8;
step_size=1;
v = 1;
tspan = [0 0.2];
max_disturbance = 0.03;

figure;
hold on;
for obstacle = obstacles.'
    sample_occlusion_space(xo, yo, obstacle(1), obstacle(2), obstacle(3));
    thetas = 0:pi/8:2*pi;
    plot(obstacle(2) + obstacle(1) * sin(thetas), obstacle(3) + obstacle(1) * cos(thetas), 'g')
end

vehicle_loc = initial_loc;
scatter(vehicle_loc(1), vehicle_loc(2), 'b')
num_robust_visibility = 0;
for i = 1:7
    [psi_actuate, is_visible] = select_next_step(vehicle_loc, psi_primitives, step_size, v, max_disturbance, obstacles, xo, yo);
    if is_visible
        num_robust_visibility = num_robust_visibility + 1;
    end
    [t,y] = ode45(@(t,y) [v*sin(y(3))+(2*max_disturbance*rand-max_disturbance);v*cos(y(3));-50*(y(3)-psi_actuate*pi/180)], tspan, vehicle_loc);
    vehicle_loc = y(end, :);
    plot(y(:, 1), y(:, 2), 'b')
end
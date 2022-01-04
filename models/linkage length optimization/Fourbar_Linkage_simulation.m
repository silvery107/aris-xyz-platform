clear all;clc;close all
% Geometry Parameters
L1 = 0.5; L2 = 0.2; x = 0.3; Lend = 1;
theta_end_1 = pi / 3;
% Variance Buffer
theta3 = 0:0.005:sqrt(-x^2 + (L1 + L2)^2);
r3 = [x * ones(1, size(theta3, 2)); -theta3];
r2 = zeros(2, size(theta3, 2));
r1 = zeros(2, size(theta3, 2));
rend = zeros(2, size(theta3, 2));
Ls = zeros(1, size(theta3, 2));

%% Simulation
for i = 1:size(theta3, 2)
    % Known theta3 calculate joint2 position
    r2(:, i) = [x; -theta3(i)];
    Ls(i) = sqrt(r2(:, i)' * r2(:, i));
    [~, c2, ~] = solve_triangle(L1, L2, Ls(i));
    theta0 = c2 + atan2(Ls(i), x);
    r1(:, i) = L1 * [cos(theta0); -sin(theta0)];
    theta_end = theta0 + theta_end_1;
    rend(:, i) = 1/2 * r1(:, i) + Lend * [cos(theta_end); -sin(theta_end)];
end

%% Visualization
for i = 1:size(r1, 2)
    scatter(rend(1, i), rend(2, i), 'LineWidth', 5)
    hold on
    xlim([-1, 1]);
    ylim([-2, 0]);
    line([0, r1(1, i)], [0, r1(2, i)], 'Linewidth', 2);
    line([r1(1, i), r2(1, i)], [r1(2, i), r2(2, i)], 'Linewidth', 2);
    line([r2(1, i), x], [r2(2, i), 0], 'Linewidth', 2);
    line([0, rend(1, i)], [0, rend(2, i)], 'Linewidth', 2)
    line([rend(1, i), r1(1, i)], [rend(2, i), r1(2, i)], 'Linewidth', 2)
    plot(rend(1, :), rend(2, :), 'LineWidth', 5)
    hold off
    pause(0.2);
end

%% Function
function [theta1, theta2, theta3] = solve_triangle(x1, x2, x3)
    theta1 = acos((x2^2 + x3^2 - x1^2) / (2 * x2 * x3));
    theta2 = acos((x1^2 + x3^2 - x2^2) / (2 * x1 * x3));
    theta3 = acos((x1^2 + x2^2 - x3^2) / (2 * x1 * x2));
end

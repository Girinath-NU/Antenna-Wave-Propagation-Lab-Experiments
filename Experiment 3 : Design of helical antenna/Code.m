clc;
clear;
close all;

% Input parameters
f = 3.5e9;
c0 = 3e8;
lambda = c0/f;

C = 0.95;        % Circumference in wavelengths
N = 10;          % Number of turns
alpha = 12;      % Pitch angle (degrees)

ground_plane_Diameter = 3*lambda;
ground_plane_radius = ground_plane_Diameter/2;

% Derived parameters
Actual_C = C * lambda;          
Radius = Actual_C / (2*pi);     
Pitch = tan(deg2rad(alpha)) * Actual_C;  
Length = Pitch * N;

% Helix generation
theta = linspace(0, 2*pi*N, 1000);
x = Radius * cos(theta);
y = Radius * sin(theta);
z = (Pitch/(2*pi)) * theta;

% Ground plane generation
ground_theta = linspace(0,2*pi,200);
ground_x = ground_plane_radius * cos(ground_theta);
ground_y = ground_plane_radius * sin(ground_theta);
ground_z = zeros(size(ground_theta));

% Plot
figure;
hold on;
fill3(ground_x, ground_y, ground_z, ...
      [0.8 0.8 0.8], 'FaceAlpha', 0.5, 'EdgeColor', 'k');

plot3(x, y, z, 'b', 'LineWidth', 2);

grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D Helical Antenna with Ground Plane');

view(45,30);
camlight;
lighting gouraud;
hold off;

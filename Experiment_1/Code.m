clear;
clc;
close all;

% Defining the Parameters
lambda = 1  % Wavelength (meters)
k = 2*pi/lambda % Wave number
D = 0.5  % Maximum dimension of the antenna (meters)
r = 100  % (far-field condition: r >> 2*D^2/lambda)
theta = linspace(0, 2*pi, 360); % Observation angles (radians)

I0 = 1; % Current amplitude
L = lambda / 2; % Dipole length (meters)

E_theta = I0 * (cos(k*L/2 * cos(theta)) - cos(k*L/2)) ./ sin(theta);
E_theta(abs(sin(theta)) < 1e-6) = 0;

% Normalization for plotting
E_theta = abs(E_theta); % Take magnitude
E_theta = E_theta / max(E_theta);

% Polar Plot of the Far-Field Pattern
figure;
polarplot(theta, E_theta, 'LineWidth', 2);
title('Normalized Far-Field Radiation Pattern');
grid on;

[phi, theta_grid] = meshgrid(linspace(0, 2*pi, 180), linspace(0, pi, 90));
E_theta_3D = I0 * (cos(k*L/2 * cos(theta_grid)) - cos(k*L/2)) ./ sin(theta_grid);
E_theta_3D(abs(sin(theta_grid)) < 1e-6) = 0; % Avoid singularities
E_theta_3D = abs(E_theta_3D); % Magnitude
E_theta_3D = E_theta_3D / max(E_theta_3D(:)); % Normalize

% Convert to Cartesian Coordinates for 3D Plot

x = E_theta_3D .* sin(theta_grid) .* cos(phi);
y = E_theta_3D .* sin(theta_grid) .* sin(phi);
z = E_theta_3D .* cos(theta_grid);
figure;
surf(x, y, z, E_theta_3D, 'EdgeColor', 'none');
title('3D Far-Field Radiation Pattern');
xlabel('X'); ylabel('Y'); zlabel('Z');
colormap jet; colorbar;
axis equal;
grid on;
view(45, 30);



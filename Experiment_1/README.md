### This MATLAB script computes and visualizes the far-field radiation pattern of a half-wave dipole antenna in both 2D (polar plot) and 3D.
### Below is a line-by-line, concept-by-concept explanation, linking the math to antenna theory.

# Housekeeping Commands
```matlab
clear;
clc;
close all;
```
   `clear`     → Removes all variables from the workspace
   
  ` clc`       → Clears the command window
   
   `close all` → Closes all open figure windows
##
# Defining Physical Parameters
```matlab
lambda = 1;      % Wavelength (meters)
k = 2*pi/lambda; % Wave number
D = 0.5;         % Maximum antenna dimension
r = 100;         % Observation distance (far-field)
theta = linspace(0, 2*pi, 360);
```
| Parameter | Meaning                                           |
| --------- | ------------------------------------------------- |
| `lambda`  | Wavelength of operation                           |
| `k`       | Propagation constant ( k = 2*pi/lambda  )         |
| `D`       | Largest antenna dimension                         |
| `r`       | Observation distance                              |
| `theta`   | Angular observation points                        |
### Farfield Check
The Franhofer Condition
##
# Dipole Antenna Parameters
```matlab
- I0 = 1;          % Current amplitude
- L = lambda / 2;  % Dipole length
```
`L` = λ/2 → Half-wave dipole

`I0` → Peak current at the dipole center

This matches the classic resonant dipole antenna.
##
# Far-Field Electric Field Equation
```matlab
E_theta = I0 * (cos(k*L/2 * cos(theta)) - cos(k*L/2)) ./ sin(theta);
```
### What this tells us:

- Radiation depends only on θ

- No radiation along the dipole axis (θ = 0°, 180°)

- Maximum radiation at θ = 90°
##
# Handling Mathematical Singularities
```matlab
E_theta(abs(sin(theta)) < 1e-6) = 0;
```
- Prevents division by zero when sin(θ) ≈ 0

- Physically correct because:

- Dipoles do not radiate along their axis
##
# Normalization
```matlab
E_theta = abs(E_theta);
E_theta = E_theta / max(E_theta);
```
- Converts to magnitude

- Normalizes peak radiation to 1
##
# 2D Polar Radiation Pattern
```matalab
figure;
polarplot(theta, E_theta, 'LineWidth', 2);
title('Normalized Far-Field Radiation Pattern');
grid on;
```
### What you see:

- Circular symmetry

- Max radiation at θ = 90°

- Nulls at θ = 0° and 180°
##
# Creating Angular Grid for 3D Plot
```matlab
[phi, theta_grid] = meshgrid(linspace(0, 2*pi, 180), linspace(0, pi, 90));
```
- theta_grid → Elevation angle (0 to π)

- phi → Azimuth angle (0 to 2π)
##
# 3D Radiation Field Computation
```matlab
E_theta_3D = I0 * (cos(k*L/2 * cos(theta_grid)) - cos(k*L/2)) ./ sin(theta_grid);
E_theta_3D(abs(sin(theta_grid)) < 1e-6) = 0;
```
- Same dipole equation

- Evaluated over θ and φ

- Still independent of φ (dipole symmetry)
##
# Normalization of 3D Pattern
```matlab
E_theta_3D = abs(E_theta_3D);
E_theta_3D = E_theta_3D / max(E_theta_3D(:));
```
- Ensures 3D surface is scaled properly

- Peak radiation = 1
## 
# Spherical → Cartesian Conversion
```matlab
x = E_theta_3D .* sin(theta_grid) .* cos(phi);
y = E_theta_3D .* sin(theta_grid) .* sin(phi);
z = E_theta_3D .* cos(theta_grid);
```
##
# 3D Radiation Pattern Plot
```matlab
figure;
surf(x, y, z, E_theta_3D, 'EdgeColor', 'none');
```
- surf → Smooth radiation surface

- Color represents field magnitude
```matab
title('3D Far-Field Radiation Pattern');
xlabel('X'); ylabel('Y'); zlabel('Z');
colormap jet; colorbar;
axis equal;
grid on;
view(45, 30);
```
### Final Result:

- Classic toroidal (donut) shape

- Zero radiation along Z-axis

- Maximum in XY-plane
##

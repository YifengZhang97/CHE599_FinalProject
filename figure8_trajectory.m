function [x, y] = figure8_trajectory(t, a)
% FIGURE8_TRAJECTORY generates horizontal figure-8 trajectory
%   t: time array
%   a: scaling factor for size (amplitude)

% Parametric equations for a horizontal figure-8 (lemniscate of sine)
x = a * sin(t);
y = a * sin(t) .* cos(t);

end
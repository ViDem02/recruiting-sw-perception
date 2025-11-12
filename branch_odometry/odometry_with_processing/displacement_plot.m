% displacement_plot.m
% Compute and plot robot displacement in the horizontal plane (East-X, North-Z)
% from an ICP transform between two point clouds.
%
% Assumptions
% - Axes: x = East, y = Up (height), z = North.
% - The R, t printed by your C++ print4x4Matrix() correspond to the rigid transform
%   used by ICP. If they map the second cloud into the first cloud frame ("second -> first"),
%   then the robot motion from first to second is the inverse transform.
% - Units of t are the same as in your point clouds (often meters).

clear; clc;

% --- INPUT: paste the R and t you printed in C++ ---
R = [
  0.996,  0.093,  0.006; 
 -0.092,  0.991, -0.093; 
 -0.015,  0.093,  0.996; 
];
t = [ -0.519;  0.249;  0.261 ];

% Set to true if the printed transform maps second -> first (as your C++ log states).
% If instead R,t map first -> second, set this to false to use t directly.
secondToFirst = true;

% New flag: invert the robot motion direction AFTER resolving transform direction.
% This is useful if you want to see the reverse traversal (e.g., going back to start).
invertRobotMotion = true;  % set true to flip motion vector

if secondToFirst
    % Given p1 = R21 * p2 + t21, motion from frame1->frame2 is the inverse:
    % T12 = inv(T21) = [R21' , -R21' * t21]
    t12 = -(R.') * t;   % translation from first to second in first-frame axes
else
    % Already first -> second
    t12 = t;
end

if invertRobotMotion
    t12 = -t12; % reverse direction
end

% Extract horizontal displacement components
east  = t12(1);  % x (East)
north = t12(3);  % z (North)

% Scalar distance in the horizontal plane
planar_dist = hypot(east, north);
heading_deg = atan2d(north, east); % 0 deg = +East, 90 deg = +North

fprintf('Displacement first -> second (horizontal):\n');
if invertRobotMotion
    fprintf('  (Inverted direction)\n');
end
fprintf('  East (x):   %+0.3f\n', east);
fprintf('  North (z):  %+0.3f\n', north);
fprintf('  Distance:    %0.3f\n', planar_dist);
fprintf('  Heading:     %0.1f deg (from +East, CCW to +North)\n', heading_deg);

% --- Plot in 2D (East vs North) ---
figure('Color','w'); clf; hold on; grid on; axis equal;

% Draw path and arrow
plot([0, east], [0, north], 'b-', 'LineWidth', 2);
quiver(0, 0, east, north, 0, 'Color', [0.85 0.2 0.2], 'LineWidth', 2, 'MaxHeadSize', 0.5);
scatter([0, east], [0, north], 40, 'filled');

xlabel('East (x)'); ylabel('North (z)');
title(sprintf('Robot displacement: East = %0.3f, North = %0.3f, Dist = %0.3f', ...
    east, north, planar_dist));
xlim padded; ylim padded;

% Annotate end point
text(east, north, sprintf('  (E = %0.3f, N = %0.3f)\\n  Heading = %0.1f^o', east, north, heading_deg), ...
    'VerticalAlignment','bottom', 'FontSize', 10);

% --- Optional: accumulate multiple steps ---
% If you have a sequence of transforms, append each (R_i, t_i) and repeat the
% inverse step as needed to get t12_i, then cumulatively sum East/North to build
% a trajectory. See the helper below for one example.
%
% Example usage:
%   Rs = {R1, R2, R3}; ts = {t1, t2, t3}; dirFlags = [true, true, true]; invFlags = [false, false, true];
%   [E, N] = accumulate_EN_displacements(Rs, ts, dirFlags, invFlags);
%   figure; plot(E, N, '-o'); axis equal; grid on; xlabel('East'); ylabel('North');

function [E, N] = accumulate_EN_displacements(Rs, ts, secondToFirstFlags, invertFlags)
    % Accumulate East/North displacements across multiple relative transforms.
    % Inputs:
    %   Rs, ts: cell arrays of 3x3 and 3x1; secondToFirstFlags: logical array (map second->first?)
    %   invertFlags: logical array to reverse motion direction after resolution
    % Outputs:
    %   E, N: vectors of cumulative East and North positions
    if nargin < 3 || isempty(secondToFirstFlags), secondToFirstFlags = true(size(Rs)); end
    if nargin < 4 || isempty(invertFlags), invertFlags = false(size(Rs)); end
    num = numel(Rs);
    E = zeros(num+1,1); N = zeros(num+1,1); % start at origin
    for k = 1:num
        Rk = Rs{k}; tk = ts{k};
        if secondToFirstFlags(k)
            dk = -(Rk.') * tk; % first->second
        else
            dk = tk;           % already first->second
        end
        if invertFlags(k)
            dk = -dk;          % reverse direction for this segment
        end
        E(k+1) = E(k) + dk(1);
        N(k+1) = N(k) + dk(3);
    end
end

%% Path Following for a Unicycle Robot
% This example demonstrates the disturbing factor effect using a Robot Simulator. 

% Copyright 2014-2016 The MathWorks, Inc. + prof Mariolino De Cecco

% BUG: The matlab "drive" function works only for x AND y > 0 (i.e. in the first quadrant)




% STEP 1 - Define Parameters
clear all;
close all;
clc;

% Baseline
b = 0.5;
% Wheel radii
R = [0.1 0.1];
% Kinematic Parameters
K_param = [b R];
DK_param = 0*[0.02 0.01 0]; % deviation from ideal parameters



% Set the current location and the goal location of the robot as defined by the path




%% --- Input data ---

Cones = [
    3.883    -3.466
    3.774    -2.478
    3.693    -1.193
    3.436    -0.084
    2.922    1.080
    2.259    2.067
    1.975    -3.574
    1.542    -2.411
    1.502    -1.856
    1.245    -1.261
    0.974    -0.814
    0.257    -0.138
    -1.163    0.484
    1.488    2.649
    0.122    3.596
    -1.055    3.920
    0.825    3.014
    2.043    2.297
    -0.257    0.214
    -1.583    0.660
    -1.975    0.836
];


x = 3;
y = -4;

robotCurrentLocation = [x y];





% === Step 1: pick two closest cones to the robot ===
d_robot = sqrt((Cones(:,1)-x).^2 + (Cones(:,2)-y).^2);
[~, order] = sort(d_robot);
first_idx = order(1);
second_idx = order(2);

% Initialize paths
path_l = Cones(first_idx, :);
path_r = Cones(second_idx, :);
remaining = true(size(Cones,1),1);
remaining([first_idx, second_idx]) = false;

current_l = first_idx;
current_r = second_idx;

angle_low = -160; 
angle_high = 160;

for step = 1:(size(Cones,1)-2)
    rem_idx = find(remaining);
    if isempty(rem_idx), break; end

    % Compute nearest candidates
    d_left  = sqrt(sum((Cones(rem_idx,:) - Cones(current_l,:)).^2, 2));
    d_right = sqrt(sum((Cones(rem_idx,:) - Cones(current_r,:)).^2, 2));

    [~, idxL] = min(d_left);
    [~, idxR] = min(d_right);
    candL = rem_idx(idxL);
    candR = rem_idx(idxR);

    % --- competition: if they chose the same cone, give it to the closer one ---
    if candL == candR
        if d_left(idxL) < d_right(idxR)
            if numel(rem_idx) > 1
                [~, orderR] = sort(d_right);
                candR = rem_idx(orderR(2));
            else
                candR = NaN;
            end
        else
            if numel(rem_idx) > 1
                [~, orderL] = sort(d_left);
                candL = rem_idx(orderL(2));
            else
                candL = NaN;
            end
        end
    end

    % --- compute turning angles (based on previous, current, next cones) ---
    % For left path
    if size(path_l,1) >= 2 && ~isnan(candL)
        prev = path_l(end-1,:);
        curr = path_l(end,:);
        next = Cones(candL,:);

        v_in  = curr - prev;
        v_out = next - curr;

        a1 = atan2(v_in(2), v_in(1));
        a2 = atan2(v_out(2), v_out(1));
        deltaL = abs(rad2deg(wrapToPi(a2 - a1)));
    else
        deltaL = 90; % neutral angle for first step
    end

    % For right path
    if size(path_r,1) >= 2 && ~isnan(candR)
        prev = path_r(end-1,:);
        curr = path_r(end,:);
        next = Cones(candR,:);

        v_in  = curr - prev;
        v_out = next - curr;

        a1 = atan2(v_in(2), v_in(1));
        a2 = atan2(v_out(2), v_out(1));
        deltaR = abs(rad2deg(wrapToPi(a2 - a1)));
    else
        deltaR = 90; % neutral
    end

    % --- check angles ---
    badL = ~isnan(deltaL) && (deltaL < angle_low || deltaL > angle_high);
    badR = ~isnan(deltaR) && (deltaR < angle_low || deltaR > angle_high);

    % --- if either bad, try swapping ---
    if badL || badR
        tmp = candL; candL = candR; candR = tmp;
        tmp = deltaL; deltaL = deltaR; deltaR = tmp;

        badL = ~isnan(deltaL) && (deltaL < angle_low || deltaL > angle_high);
        badR = ~isnan(deltaR) && (deltaR < angle_low || deltaR > angle_high);

        if badL || badR
            % If still bad, we can skip updating that side this round
            if badL, candL = NaN; end
            if badR, candR = NaN; end
        end
    end

    % --- assign and update ---
    if ~isnan(candL)
        path_l = [path_l; Cones(candL,:)];
        remaining(candL) = false;
        current_l = candL;
    end
    if ~isnan(candR)
        path_r = [path_r; Cones(candR,:)];
        remaining(candR) = false;
        current_r = candR;
    end
end

disp("Left path (coordinates):")
disp(path_l)
disp("Right path (coordinates):")
disp(path_r)



%% Compute Racing Line
if size(path_l,1) ~= size(path_r,1)
    % interpolate shorter one to match
    n = min(size(path_l,1), size(path_r,1));
    sL = linspace(0,1,size(path_l,1));
    sR = linspace(0,1,size(path_r,1));
    s  = linspace(0,1,n);
    path_l = interp1(sL, path_l, s, 'linear');
    path_r = interp1(sR, path_r, s, 'linear');
end

% Midpoints between left and right cones
racing_line = (path_l + path_r) / 2;

% Optional: smooth it to reduce zigzags
window = 5;  % adjust for more or less smoothing
racing_line(:,1) = smooth(racing_line(:,1), window);
racing_line(:,2) = smooth(racing_line(:,2), window);

% Plot result
figure; hold on; axis equal; grid on;
plot(path_l(:,1), path_l(:,2), 'r-*', 'DisplayName','Left Path');
plot(path_r(:,1), path_r(:,2), 'b-*', 'DisplayName','Right Path');
plot(racing_line(:,1), racing_line(:,2), 'k-', 'LineWidth',2, 'DisplayName','Racing Line');
legend;
title('Racing Line Between Cones');


% Robot heading
theta0 = 0;
% wheel encoder resolution [tic/rev]
Enc_res = 4096;

% Robot heading error
theta0 = theta0 + 0*pi/180;


% Assume an initial robot orientation [rad]
initialOrientation = pi/2;

% Define the current pose for the robot [x y theta]
robotInitialPose = [robotCurrentLocation initialOrientation];

% Initialize/Construct the Robot Simulator
% The inputs are: linear and angular velocities like a unicycle

robot = DiffDriveSimulator(K_param, DK_param, Enc_res, robotInitialPose, path_l, path_r);
robot.setRobotPose(robotInitialPose);

Pose0 = robot.GrabPose; % initial pose



% 
% 
% %% STEP 2a - Simulate the robot behaviour with Encoder Tics/cycle
% % --- input cones (same as yours) ---
% Cones = [
%     0 0
%     0 1
%     1 1 
%     1 0
%     0 2
%    -1 1];
% 
% % --- robot position: must exist in workspace as x,y; otherwise default to [0,0] ---
% if ~(exist('x','var') && exist('y','var'))
%     warning('Variables x,y not found. Assuming robot at [0,0].');
%     x = 0; y = 0;
% end
% 
% N = size(Cones,1);
% 
% % distances from robot to each cone
% d_robot = sqrt( (Cones(:,1)-x).^2 + (Cones(:,2)-y).^2 );
% [sortedD, order] = sort(d_robot);
% 
% % pick two closest cones as starting points
% first_idx  = order(1);
% second_idx = order(2);
% 
% % remaining mask (true = still to visit)
% remaining = true(N,1);
% remaining(first_idx)  = false;
% remaining(second_idx) = false;
% 
% % initialize paths (store coordinates and optionally original indices later)
% path_l = Cones(first_idx, :);   % left path starts at first_idx
% path_r = Cones(second_idx, :);  % right path starts at second_idx
% 
% current_left  = first_idx;
% current_right = second_idx;
% 
% % optional: angular constraint (degrees). Set to [] to disable.
% angle_limit = 20;  % similar to your code: disallow near-collinear > 180-20 or < 20 deg
% use_angle_check = true;
% 
% for step = 1:(N-2)
%     rem_idx = find(remaining);
%     if isempty(rem_idx), break; end
% 
%     % distances from current left to remaining cones
%     d_left = sqrt( sum( (Cones(rem_idx,:) - Cones(current_left,:)).^2, 2 ) );
%     [dminL, indL] = min(d_left);
%     cand_left = rem_idx(indL);
% 
%     % distances from current right to remaining cones
%     d_right = sqrt( sum( (Cones(rem_idx,:) - Cones(current_right,:)).^2, 2 ) );
%     [dminR, indR] = min(d_right);
%     cand_right = rem_idx(indR);
% 
%     % if both sides picked the same cone, give the right path the next-best candidate
%     if cand_left == cand_right
%         if numel(rem_idx) >= 2
%             % pick second best for the right side
%             [~, sortedIdxs] = sort(d_right);
%             if numel(sortedIdxs) >= 2
%                 cand_right = rem_idx(sortedIdxs(2));
%             else
%                 cand_right = NaN;
%             end
%         else
%             cand_right = NaN;
%         end
%     end
% 
%     % optional: angle check (replicates idea in your original code)
%     if use_angle_check
%         % compute vectors and angles in degrees for both candidate vectors
%         % guard for NaN
%         ok_swap = false;
%         if ~isnan(cand_left)
%             vL = Cones(cand_left,:) - Cones(current_left,:);
%             theta1 = cart2pol(vL(1), vL(2)) * 180/pi;
%         else
%             theta1 = NaN;
%         end
%         if ~isnan(cand_right)
%             vR = Cones(cand_right,:) - Cones(current_right,:);
%             theta2 = cart2pol(vR(1), vR(2)) * 180/pi;
%         else
%             theta2 = NaN;
%         end
% 
%         % normalize angles to [0,360)
%         if ~isnan(theta1), theta1 = mod(theta1,360); end
%         if ~isnan(theta2), theta2 = mod(theta2,360); end
% 
%         % check "bad" condition similar to original: angle near 0 or near 180 (collinear-ish)
%         bad1 = ~isnan(theta1) && (theta1 < angle_limit || theta1 > 360-angle_limit || abs(theta1-180) < angle_limit);
%         bad2 = ~isnan(theta2) && (theta2 < angle_limit || theta2 > 360-angle_limit || abs(theta2-180) < angle_limit);
% 
%         % If either is bad, try swapping candidates (attempt)
%         if (bad1 || bad2) && ~isnan(cand_right) && ~isnan(cand_left)
%             % attempt swap: pick the other candidate's nearest for the respective side
%             % (a pragmatic attempt — if it still fails we will accept it or skip)
%             % swap candidates
%             tmp = cand_left; cand_left = cand_right; cand_right = tmp;
%             % recompute angles after swap
%             if ~isnan(cand_left)
%                 vL = Cones(cand_left,:) - Cones(current_left,:);
%                 theta1 = mod(cart2pol(vL(1), vL(2)) * 180/pi, 360);
%             end
%             if ~isnan(cand_right)
%                 vR = Cones(cand_right,:) - Cones(current_right,:);
%                 theta2 = mod(cart2pol(vR(1), vR(2)) * 180/pi, 360);
%             end
%             bad1 = ~isnan(theta1) && (theta1 < angle_limit || theta1 > 360-angle_limit || abs(theta1-180) < angle_limit);
%             bad2 = ~isnan(theta2) && (theta2 < angle_limit || theta2 > 360-angle_limit || abs(theta2-180) < angle_limit);
%             if bad1 || bad2
%                 % If still bad, continue anyway (or you can choose to error)
%                 % here we just continue and accept the candidate.
%             end
%         end
%     end
% 
%     % append chosen candidates to paths and remove from remaining
%     if ~isnan(cand_left)
%         path_l = [path_l; Cones(cand_left,:)];
%         remaining(cand_left) = false;
%         current_left = cand_left;
%     end
%     if ~isnan(cand_right)
%         path_r = [path_r; Cones(cand_right,:)];
%         remaining(cand_right) = false;
%         current_right = cand_right;
%     end
% end
% 
% disp('Left path (coordinates):')
% disp(path_l)
% disp('Right path (coordinates):')
% disp(path_r)
% 





%% STEP 2a - Simulate the robot behaviour with Encoder Tics/cycle

% Define the total simulation time [s]
Tt = 10;

% Define the While Loop update time [s]:
Tc = 0.05;
% Define the current time [s]:
T = 0;

%NTic_l = 10 * ones(1,204);
%NTic_l = [NTic -NTic];

%NTic_r = -10 * ones(1,204);

% pose = robot.GrabPose;
% 
% x = pose(1);
% y = pose(2);
% 
% Cones = [
%     0 0
%     0 1
%     1 1 
%     1 0
%     0 2
%    -1 1];
% 
% distances = zeros(length(Cones), 2);
% 
% for i=1:length(Cones)
%     d = sqrt((Cones(i,1)-x)^2 + (Cones(i,2)-y)^2);
%     distances(i, 1) = i;
%     distances(i, 2) = d;
% end
% 
% 
% sortrows(distances, 2);
% first_cone_idx = distances(1,1);
% second_cone_idx = distances(2,1);
% 
% path_l = [];
% path_r = [];
% 
% Candidates = [Cones,(1:length(Cones))',(1:length(Cones))'];
% Candidates(first_cone_idx, :) = [];
% Candidates(second_cone_idx, :) = [];
% 
% for jjj=1:length(Cones)-2
%     x = Cones(first_cone_idx, 1);
%     y = Cones(first_cone_idx, 2);
% 
%     Candidates(:, 4) = (1:size(Candidates,1))';
% 
%     distances_first = [inf(size(Candidates,1), 1),Candidates(:,3),Candidates(:, 4)];
% 
%     for i=1:length(Candidates)
%         d = sqrt((Candidates(i,1)-x)^2 + (Candidates(i,2)-y)^2);
%         distances_first(i, 1) = d;
%     end
% 
%     sortrows(distances_first, 1);
% 
%     if length(distances_first) >= 1
%         good_candidate_first_idx = distances_first(1,2);
%         next_first = distances_first(1,3);
%         Candidates(Candidates(:,3)==next_first, :) = [];
%     else
%         good_candidate_first_idx = NaN;
%     end
% 
%     if ~isnan(good_candidate_first_idx)
% 
%         x = Cones(second_cone_idx, 1);
%         y = Cones(second_cone_idx, 2);
%         distances_second = [inf(size(Candidates,1), 1),Candidates(:,3),Candidates(:, 4)];
% 
%         for i=1:length(Candidates)
%             d = sqrt((Cones(i,1)-x)^2 + (Cones(i,2)-y)^2);
%             distances_second(i, 1) = d;
%         end
% 
%         sortrows(distances_second, 1);
% 
%         if length(distances_second) >= 1
%             good_candidate_second_idx = distances_second(1,2);
%             next_second = distances_first(1,3);
%             Candidates(next_second, :) = [];
%         else
%             good_candidate_second_idx = NaN;
%         end
% 
%     else
% 
%         good_candidate_second_idx = NaN;
% 
%     end
% 
%     % check
% 
%     temp = NaN;
% 
%     if ~isnan(good_candidate_first_idx) || ~isnan(good_candidate_second_idx)
%         if ~isnan(good_candidate_first_idx)
%             x_gc_f_respect_to_first = Cones(good_candidate_first_idx, 1) - Cones(first_cone_idx, 1);
%             y_gc_f_respect_to_first = Cones(good_candidate_first_idx, 2) - Cones(first_cone_idx, 2);
%         end
% 
%         if ~isnan(good_candidate_second_idx)
%             x_gc_f_respect_to_second = Cones(good_candidate_second_idx, 1) - Cones(second_cone_idx, 1);
%             y_gc_f_respect_to_second = Cones(good_candidate_second_idx, 2) - Cones(second_cone_idx, 2);
%         end
% 
%         tetha1 = cart2pol(x_gc_f_respect_to_first, y_gc_f_respect_to_first) * (180/pi);
%         tetha2 = cart2pol(x_gc_f_respect_to_second, y_gc_f_respect_to_second) * (180/pi);
% 
%         if (tetha1 > (180-20) || tetha1 < (20)) || (tetha2 > (180-20) || tetha2 < (20))
%             temp = good_candidate_first_idx;
%             good_candidate_second_idx = good_candidate_first_idx;
%             good_candidate_first_idx = temp;
%             temp = next_first;
%             next_first = next_second;
%             next_second = temp;
% 
%             if ~isnan(good_candidate_first_idx)
%                 x_gc_f_respect_to_first = Cones(good_candidate_first_idx, 1) - Cones(first_cone_idx, 1);
%                 y_gc_f_respect_to_first = Cones(good_candidate_first_idx, 2) - Cones(first_cone_idx, 2);
%             end
% 
%             if ~isnan(good_candidate_second_idx)
%                 x_gc_f_respect_to_second = Cones(good_candidate_second_idx, 1) - Cones(second_cone_idx, 1);
%                 y_gc_f_respect_to_second = Cones(good_candidate_second_idx, 2) - Cones(second_cone_idx, 2);
%             end
% 
%             tetha1 = cart2pol(x_gc_f_respect_to_first, y_gc_f_respect_to_first) * (180/pi);
%             tetha2 = cart2pol(x_gc_f_respect_to_second, y_gc_f_respect_to_second) * (180/pi);
% 
%             if (tetha1 > (180-20) || tetha1 < (20)) || (tetha2 > (180-20) || tetha2 < (20))
%                 error("no solution found")
%             end 
%         end
%     end
% 
%     if ~isnan(good_candidate_first_idx)
%         path_l = [path_l; Cones(good_candidate_first_idx, :)];
%     end 
% 
%     if ~isnan(good_candidate_second_idx)
%         path_r = [path_r; Cones(good_candidate_second_idx, :)];
%     end
% 
%     first_cone_idx = next_first;
%     second_cone_idx = next_second;
% 
%     if isnan(first_cone_idx) || isnan(second_cone_idx)
%         break
%     end
% 
% end
% 
% disp(path_l)
% disp(path_r)
% 
% return
% 



tic_fwd = 650;
tic_turn = 255;
coefficient_l = 10;
coefficient_r = 10;

NTic_l = [];
NTic_r = [];

for i=1:4
    NTic_l = [NTic_l, coefficient_l * ones(1,tic_fwd)];
    NTic_r = [NTic_r, coefficient_r * ones(1,tic_fwd)];

    NTic_l = [NTic_l, -coefficient_l * ones(1,tic_turn)];
    NTic_r = [NTic_r, coefficient_r * ones(1,tic_turn)];
end
% 
% %NTic_l = [coefficient * ones(1,tic_fwd), -coefficient * ones(1,tic_turn), coefficient * ones(1,tic_fwd), -coefficient * ones(1,tic_turn), coefficient * ones(1,tic_fwd), -coefficient * ones(1,tic_turn), coefficient * ones(1,tic_fwd), -coefficient * ones(1,tic_turn)];
% %NTic_r = [coefficient * ones(1,tic_fwd),  coefficient * ones(1,tic_turn), coefficient * ones(1,tic_fwd),  coefficient * ones(1,tic_turn), coefficient * ones(1,tic_fwd),  coefficient * ones(1,tic_turn), coefficient * ones(1,tic_fwd), coefficient * ones(1,tic_turn)];

% Define the desired/ideal path


Vr = K_param(2) * 2*pi * NTic_r(2) / (Enc_res * Tc);
path = [0.00    0.00;
    Vr * Tt/2    0];

Nplot = 10; % plot number
Xlim = [-5 5];
Ylim = [-5 5];

if length(NTic_l) ~= length(NTic_l)
    error("length are different")
end


% x = [-4.5]; 
% y = [-4.5];
% 
% for i=1:length(x)
%     robot.setRobotPose([x(i), y(i), 0]);
%     robot.Show(path, Nplot, Xlim, Ylim);
%     pause(1);
% end
% 

robot.Show(path, Nplot, Xlim, Ylim);

delete(robot)

return


% Simulates the robot trajectory:
for i=1:length(NTic_l)
     
    % MEASUREMENT: acquire the controller outputs, i.e., the inputs to the robot
    
    % Tic of the encoder wheels
    NTic_r_curr = NTic_r(i);
    NTic_l_curr = NTic_l(i);
    
    % SIMULATION: Simulate the robot using the controller outputs.
    robot.SimulateEnc(Tc, NTic_r_curr, NTic_l_curr);
    robot.Show(path, Nplot, Xlim, Ylim);
    
%     % PERCEPTION: Extract current location information ([X,Y]) from the 
%     % current pose of the robot and ADD NOISE simulating the sensors inaccuracy
%     robotCurrentPose = robot.robotCurrentPose + ...
%         0 * [normrnd(0, 0.01, 1, 2) normrnd(0, 2)*pi/180];
    
    % Re-compute the current time
    T = T + Tc;
        
    waitfor(Tc);
 
end

Pose = robot.GrabPose


% Close simulation.
delete(robot)



%% STEP 2b - Simulate the robot behaviour with wheels velocity

% Robot (ideal) linear [left right] wheels velocity
Vr = 1;
Vl = 0.9;

% Define the total simulation time [s]
Tt = 10;
% Define the While Loop update time [s]:
Tc = 0.05;
% Define the current time [s]:
T = 0;

% Define the desired/ideal path
path = [0.00    0.00;
    Vr * Tt    0];

Nplot = 10; % plot number
Xlim = [-1 11];
Ylim = [-6 6];

% Simulates the robot control LOOP:
while( T < Tt )
     
    % ACTION: Compute the controller outputs, i.e., the inputs to the robot
    
    % Wheels Velocity
    
    % Simulate the robot using the controller outputs.
    robot.Simulate(Tc, Vr, Vl);
    robot.Show(path, Nplot, Xlim, Ylim);
    
%     % PERCEPTION: Extract current location information ([X,Y]) from the 
%     % current pose of the robot and ADD NOISE simulating the sensors inaccuracy
%     robotCurrentPose = robot.robotCurrentPose + ...
%         0 * [normrnd(0, 0.01, 1, 2) normrnd(0, 2)*pi/180];
    
    % Re-compute the current time
    T = T + Tc;
    
    waitfor(Tc);
 
end

% Close simulation.
delete(robot)




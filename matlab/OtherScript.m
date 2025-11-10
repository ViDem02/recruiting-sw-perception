Cones = [
    1.989    -4.683
    2.043    -4.413
    2.056    -3.912
    2.083    -3.439
    2.205    -2.532
    2.205    -2.316
    2.232    -1.369
    2.340    -0.097
    2.232    -0.381
    2.232    -0.990
    2.246    -1.721
    2.137    -3.006
    3.869    -4.359
    3.910    -3.953
    3.950    -3.236
    3.950    -2.708
    3.950    -2.397
    3.964    -1.991
    3.977    -1.504
    4.004    -1.234
    4.113    -0.070
    4.126    0.187
    4.153    -0.733
    4.045    -0.138
    2.543    0.728
    4.518    0.836
    4.410    0.714
    4.424    1.269
    4.573    1.688
    4.559    2.027
    4.532    2.311
    4.505    2.446
    4.315    2.744
    4.140    2.919
    3.842    3.136
    3.680    3.231
    2.665    3.596
    2.394    3.650
    2.909    3.582
    3.220    3.460
    1.948    3.501
    1.596    3.122
    1.393    2.919
    1.272    2.568
    1.136    2.121
    1.001    1.837
    0.825    1.296
    2.557    1.039
    2.246    0.985
    2.124    0.687
    2.043    0.525
    1.975    0.308
    1.853    -0.043
    1.772    -0.449
    1.650    -0.963
    1.366    -1.883
    1.326    -2.100
    1.096    -2.938
    0.879    -3.506
    0.230    -4.237
    -0.771    -4.196
    -1.623    -3.723
    -2.665    -2.059
    -2.760    -1.261
    -3.166    0.335
    0.460    0.985
    0.568    0.484
    0.501    0.295
    0.379    -0.138
    0.257    -0.530
    0.135    -0.855
    0.054    -1.139
    -0.189    -2.005
    -0.298    -2.302
    -0.866    -2.438
    -0.879    -2.330
    -1.245    -1.342
    -1.245    -0.611
    -1.353    -0.341
    -1.353    0.173
    -1.380    0.538
    -1.448    0.850
    -1.069    -1.843
    -1.299    -0.977
    -0.203    -1.518
    0.663    0.809
    0.893    1.445
    1.650    -1.166
    1.502    -1.545
    1.475    -1.707
    1.718    -0.544
    1.272    -2.573
    1.231    -2.817
    1.001    -3.534
    0.798    -3.723
    0.663    -3.899
    0.582    -3.980
    0.230    -4.088
    0.095    -4.196
    -0.365    -4.223
    -0.663    -4.210
    -0.988    -4.115
    -1.312    -3.912
    -1.407    -3.791
    -1.556    -3.588
    -1.880    -3.331
    -2.205    -3.074
    -2.300    -2.938
    -2.422    -2.262
    -2.449    -2.032
    -2.462    -2.641
    -2.462    -2.465
    -2.462    -1.694
    -2.530    -1.585
    -2.584    -1.423
    -2.652    -1.261
    -2.746    -0.963
    -2.760    -0.882
    -2.895    -0.503
    -2.949    -0.152
    -3.057    0.295
];




%% Parameters
robot_pos =  [2.8, -6];  % start roughly below center
robot_heading = pi/2;               % facing upward
max_angle = pi/2;                   % only consider cones within ±90°
step_limit = 200;                   % safety limit
plot_delay = 0.1;                   % seconds between frames

%% Initialize paths
path_center = robot_pos;
path_left = [];
path_right = [];

% Plot start
scatter(robot_pos(1), robot_pos(2), 80, 'go', 'filled');
drawnow;

% %% Main Greedy Loop
% for step = 1:step_limit
%     if isempty(Cones)
%         break;
%     end
% 
%     rel = Cones - robot_pos;
%     dists = sqrt(sum(rel.^2,2));
%     angles = atan2(rel(:,2), rel(:,1)) - robot_heading;
%     angles = wrapToPi(angles);
% 
%     visible_idx = find(abs(angles) < max_angle);
%     if isempty(visible_idx)
%         break;
%     end
% 
%     left_candidates = visible_idx(angles(visible_idx) > 0);
%     right_candidates = visible_idx(angles(visible_idx) < 0);
% 
%     if isempty(left_candidates) || isempty(right_candidates)
%         break;
%     end
% 
%     [~, iL] = min(dists(left_candidates));
%     [~, iR] = min(dists(right_candidates));
%     left_cone = Cones(left_candidates(iL), :);
%     right_cone = Cones(right_candidates(iR), :);
% 
%     % Midpoint = next robot position (center of track)
%     mid = (left_cone + right_cone)/2;
% 
%     % Store paths
%     path_left = [path_left; left_cone];
%     path_right = [path_right; right_cone];
%     path_center = [path_center; mid];
% 
%     % Update state
%     robot_heading = atan2(mid(2)-robot_pos(2), mid(1)-robot_pos(1));
%     robot_pos = mid;
% 
%     % Remove used cones
%     Cones([left_candidates(iL), right_candidates(iR)], :) = [];
% 
%     % Visualization
%     clf; hold on; grid on; axis equal;
%     title(sprintf('Step %d', step));
%     scatter(Cones(:,1), Cones(:,2), 60, [0.8 0.8 0.8], 'filled');
%     plot(path_left(:,1), path_left(:,2), 'r-o', 'LineWidth', 2);
%     plot(path_right(:,1), path_right(:,2), 'b-o', 'LineWidth', 2);
%     plot(path_center(:,1), path_center(:,2), 'k--', 'LineWidth', 2);
%     scatter(robot_pos(1), robot_pos(2), 80, 'go', 'filled');
%     drawnow;
%     pause(plot_delay);
% end
% 
% 
% %% Main Greedy Loop (safety-aware)
% for step = 1:step_limit
%     if isempty(Cones)
%         break;
%     end
% 
%     rel = Cones - robot_pos;
%     dists = sqrt(sum(rel.^2,2));
%     angles = atan2(rel(:,2), rel(:,1)) - robot_heading;
%     angles = wrapToPi(angles);
% 
%     % only consider cones roughly in front of robot
%     visible_idx = find(abs(angles) < max_angle);
%     if isempty(visible_idx)
%         break;
%     end
% 
%     left_candidates = visible_idx(angles(visible_idx) > 0);
%     right_candidates = visible_idx(angles(visible_idx) < 0);
% 
%     if isempty(left_candidates) || isempty(right_candidates)
%         break;
%     end
% 
%     best_score = inf;
%     best_L = NaN;
%     best_R = NaN;
%     best_mid = [NaN, NaN];
% 
%     % Try all reasonable left-right pairs and pick the one with best geometry
%     for iL = 1:length(left_candidates)
%         for iR = 1:length(right_candidates)
%             L = Cones(left_candidates(iL), :);
%             R = Cones(right_candidates(iR), :);
% 
%             mid = (L + R)/2;
%             corridor_width = norm(L - R);
% 
%             % Require corridor not too narrow
%             if corridor_width < 0.3, continue; end
% 
%             % Angle to midpoint
%             heading_to_mid = atan2(mid(2)-robot_pos(2), mid(1)-robot_pos(1));
%             dtheta = abs(wrapToPi(heading_to_mid - robot_heading));
% 
%             % Distance ahead
%             dist_to_mid = norm(mid - robot_pos);
% 
%             % Skip if this is behind or a sharp U-turn (>120°)
%             if dtheta > 2*pi/3
%                 continue;
%             end
% 
%             % Score: prefer closer, well-aligned, gentle corridor
%             score = 1.0*dist_to_mid + 0.5*dtheta + 0.2*abs(corridor_width - 1.0);
% 
%             if score < best_score
%                 best_score = score;
%                 best_L = left_candidates(iL);
%                 best_R = right_candidates(iR);
%                 best_mid = mid;
%             end
%         end
%     end
% 
%     if isnan(best_L) || isnan(best_R)
%         disp('🚧 No safe corridor ahead — stopping.');
%         break;
%     end
% 
%     left_cone = Cones(best_L, :);
%     right_cone = Cones(best_R, :);
%     mid = best_mid;
% 
%     % Store paths
%     path_left = [path_left; left_cone];
%     path_right = [path_right; right_cone];
%     path_center = [path_center; mid];
% 
%     % Update robot
%     robot_heading = atan2(mid(2)-robot_pos(2), mid(1)-robot_pos(1));
%     robot_pos = mid;
% 
%     % Remove used cones
%     Cones([best_L, best_R], :) = [];
% 
%     % Visualization
%     clf; hold on; grid on; axis equal;
%     title(sprintf('Step %d', step));
%     scatter(Cones(:,1), Cones(:,2), 60, [0.8 0.8 0.8], 'filled');
%     plot(path_left(:,1), path_left(:,2), 'r-o', 'LineWidth', 2);
%     plot(path_right(:,1), path_right(:,2), 'b-o', 'LineWidth', 2);
%     plot(path_center(:,1), path_center(:,2), 'k--', 'LineWidth', 2);
%     scatter(robot_pos(1), robot_pos(2), 80, 'go', 'filled');
%     drawnow;
%     pause(plot_delay);
% end
% 
% 
% %% Main Greedy Loop (tight-bend robust)
% for step = 1:step_limit
%     if isempty(Cones)
%         break;
%     end
% 
%     rel = Cones - robot_pos;
%     dists = sqrt(sum(rel.^2,2));
%     angles = atan2(rel(:,2), rel(:,1)) - robot_heading;
%     angles = wrapToPi(angles);
% 
%     visible_idx = find(abs(angles) < max_angle);
%     if isempty(visible_idx)
%         break;
%     end
% 
%     left_candidates = visible_idx(angles(visible_idx) > 0);
%     right_candidates = visible_idx(angles(visible_idx) < 0);
%     if isempty(left_candidates) || isempty(right_candidates)
%         break;
%     end
% 
%     best_score = inf;
%     best_L = NaN;
%     best_R = NaN;
%     best_mid = [NaN, NaN];
% 
%     for iL = 1:length(left_candidates)
%         for iR = 1:length(right_candidates)
%             L = Cones(left_candidates(iL), :);
%             R = Cones(right_candidates(iR), :);
% 
%             mid = (L + R)/2;
%             width = norm(L - R);
%             dist_mid = norm(mid - robot_pos);
% 
%             % Skip if midpoint is behind robot
%             heading_to_mid = atan2(mid(2)-robot_pos(2), mid(1)-robot_pos(1));
%             dtheta = abs(wrapToPi(heading_to_mid - robot_heading));
%             if dtheta > deg2rad(150), continue; end
% 
%             % Corridor width penalty: prefer near 1.0 m but tolerate tight bends
%             width_penalty = abs(width - 1.0);
%             if width < 0.25, continue; end
% 
%             % Angular smoothness: prefer midpoints roughly ahead
%             heading_penalty = dtheta;
% 
%             % Adaptive step: prefer midpoints not too close or far
%             dist_penalty = abs(dist_mid - 0.5); % target ~0.5m ahead
% 
%             score = 1.0*dist_penalty + 1.5*heading_penalty + 0.3*width_penalty;
% 
%             if score < best_score
%                 best_score = score;
%                 best_L = left_candidates(iL);
%                 best_R = right_candidates(iR);
%                 best_mid = mid;
%             end
%         end
%     end
% 
%     if isnan(best_L) || isnan(best_R)
%         disp('🚧 No feasible path ahead — stopping.');
%         break;
%     end
% 
%     left_cone = Cones(best_L, :);
%     right_cone = Cones(best_R, :);
%     mid = best_mid;
% 
%     % Store
%     path_left = [path_left; left_cone];
%     path_right = [path_right; right_cone];
%     path_center = [path_center; mid];
% 
%     % Smooth heading update — turns gently
%     desired_heading = atan2(mid(2)-robot_pos(2), mid(1)-robot_pos(1));
%     robot_heading = robot_heading + 0.5 * wrapToPi(desired_heading - robot_heading);
% 
%     % Move toward midpoint adaptively
%     turn_severity = abs(wrapToPi(desired_heading - robot_heading));
%     step_size = 0.4 - 0.2 * (turn_severity / (pi/2)); % smaller steps for tight turns
%     robot_pos = robot_pos + step_size * [cos(robot_heading), sin(robot_heading)];
% 
%     % Remove cones already passed (closest to robot)
%     rem_dists = sqrt(sum((Cones - robot_pos).^2,2));
%     Cones(rem_dists < 0.3, :) = [];
% 
%     % Visualization
%     clf; hold on; grid on; axis equal;
%     title(sprintf('Step %d', step));
%     scatter(Cones(:,1), Cones(:,2), 60, [0.8 0.8 0.8], 'filled');
%     plot(path_left(:,1), path_left(:,2), 'r-o', 'LineWidth', 2);
%     plot(path_right(:,1), path_right(:,2), 'b-o', 'LineWidth', 2);
%     plot(path_center(:,1), path_center(:,2), 'k--', 'LineWidth', 2);
%     scatter(robot_pos(1), robot_pos(2), 80, 'go', 'filled');
%     drawnow;
%     pause(plot_delay);
% end



%% Main Greedy Loop (supports 180° hairpins)
for step = 1:step_limit
    if isempty(Cones)
        break;
    end

    rel = Cones - robot_pos;
    dists = sqrt(sum(rel.^2,2));
    angles = atan2(rel(:,2), rel(:,1)) - robot_heading;
    angles = wrapToPi(angles);

    % Widen field of view — allow cones up to 150° behind
    visible_idx = find(abs(angles) < deg2rad(150));
    if isempty(visible_idx)
        break;
    end

    left_candidates = visible_idx(angles(visible_idx) > 0);
    right_candidates = visible_idx(angles(visible_idx) < 0);
    if isempty(left_candidates) || isempty(right_candidates)
        break;
    end

    best_score = inf;
    best_L = NaN;
    best_R = NaN;
    best_mid = [NaN, NaN];

    for iL = 1:length(left_candidates)
        for iR = 1:length(right_candidates)
            L = Cones(left_candidates(iL), :);
            R = Cones(right_candidates(iR), :);
            mid = (L + R)/2;
            width = norm(L - R);
            dist_mid = norm(mid - robot_pos);

            % Corridor must be realistic
            if width < 0.2 || width > 3.0, continue; end

            % Compute direction to midpoint
            heading_to_mid = atan2(mid(2)-robot_pos(2), mid(1)-robot_pos(1));
            dtheta = abs(wrapToPi(heading_to_mid - robot_heading));

            % Compute "corridor alignment": left–right orientation
            corridor_dir = atan2(L(2)-R(2), L(1)-R(1)) + pi/2;
            d_corridor = abs(wrapToPi(corridor_dir - robot_heading));

            % Penalize misaligned midpoints unless turning is gradual
            heading_penalty = 0.5 * dtheta + 0.5 * d_corridor;

            % Prefer distances in a moderate range (0.3–1.0 m)
            dist_penalty = abs(dist_mid - 0.5);

            score = 1.0*dist_penalty + 1.0*heading_penalty + 0.3*abs(width - 1.0);

            if score < best_score
                best_score = score;
                best_L = left_candidates(iL);
                best_R = right_candidates(iR);
                best_mid = mid;
            end
        end
    end

    if isnan(best_L) || isnan(best_R)
        disp('🚧 No valid corridor found — stopping.');
        break;
    end

    left_cone = Cones(best_L, :);
    right_cone = Cones(best_R, :);
    mid = best_mid;

    % Save path
    path_left = [path_left; left_cone];
    path_right = [path_right; right_cone];
    path_center = [path_center; mid];

    % Smooth heading adjustment — allows 180° turns gradually
    desired_heading = atan2(mid(2)-robot_pos(2), mid(1)-robot_pos(1));
    delta_heading = wrapToPi(desired_heading - robot_heading);
    robot_heading = robot_heading + 0.6 * delta_heading;  % blend turn

    % Adaptive step size — smaller when turning sharply
    turn_factor = 1 - min(abs(delta_heading) / pi, 1);
    step_size = 0.5 * turn_factor + 0.15; % 0.65 straight → 0.15 tight turn

    robot_pos = robot_pos + step_size * [cos(robot_heading), sin(robot_heading)];

    % Remove nearby cones
    rem_dists = sqrt(sum((Cones - robot_pos).^2,2));
    Cones(rem_dists < 0.25, :) = [];

    % Visualization
    clf; hold on; grid on; axis equal;
    title(sprintf('Step %d', step));
    scatter(Cones(:,1), Cones(:,2), 60, [0.8 0.8 0.8], 'filled');
    plot(path_left(:,1), path_left(:,2), 'r-o', 'LineWidth', 2);
    plot(path_right(:,1), path_right(:,2), 'b-o', 'LineWidth', 2);
    plot(path_center(:,1), path_center(:,2), 'k--', 'LineWidth', 2);
    scatter(robot_pos(1), robot_pos(2), 80, 'go', 'filled');
    drawnow;
    pause(plot_delay);
end





%% Final Result
disp('✅ Simulation complete.');
disp('Left path:'); disp(path_left);
disp('Right path:'); disp(path_right);
disp('Centerline:'); disp(path_center);

figure('Name','Final Path','NumberTitle','off');
hold on; axis equal; grid on;
plot(path_left(:,1), path_left(:,2), 'r-o', 'LineWidth', 2);
plot(path_right(:,1), path_right(:,2), 'b-o', 'LineWidth', 2);
plot(path_center(:,1), path_center(:,2), 'k--', 'LineWidth', 2);
scatter(path_center(1,1), path_center(1,2), 100, 'go', 'filled');
title('Greedy Centerline Path Simulation');
legend('Left cones','Right cones','Centerline','Robot start');

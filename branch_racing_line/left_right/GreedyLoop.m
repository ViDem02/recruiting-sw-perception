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

% STEP 1 - Define Parameters
clear all;


%% --- Input data ---

Cones = [
-0.66185 4.8492
0.594821 4.87921
0.356828 5.97727
0.548648 7.32776
-0.927375 6.8815
1.14332 6.51109
];


Cones = [
-0.66185 4.8492 %
0.594821 4.87921
0.356828 5.97727
0.548648 7.32776
-0.927375 6.8815 %
1.14332 6.51109 %
];




Cones(:,2) =  (Cones(:,2) );

x = +0.5;
y = -0.5;

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

% Angle constraints: corridor turn angle must lie within [-45°, +45°]
angle_low = -45; 
angle_high = 45;

for step = 1:(size(Cones,1)-2)
    rem_idx = find(remaining);
    if isempty(rem_idx), break; end

    % If only one cone is left, assign it to the side that yields the smallest turn
    if numel(rem_idx) == 1
        lastIdx = rem_idx(1);

        % Compute signed turn angle if appended to LEFT path
        if size(path_l,1) >= 2
            prev = path_l(end-1,:);
            curr = path_l(end,:);
            next = Cones(lastIdx,:);
            v_in  = curr - prev;
            v_out = next - curr;
            a1 = atan2(v_in(2), v_in(1));
            a2 = atan2(v_out(2), v_out(1));
            deltaIfLeft = rad2deg(wrapToPi(a2 - a1));
        else
            deltaIfLeft = 0; % neutral if not enough history
        end

        % Compute signed turn angle if appended to RIGHT path
        if size(path_r,1) >= 2
            prev = path_r(end-1,:);
            curr = path_r(end,:);
            next = Cones(lastIdx,:);
            v_in  = curr - prev;
            v_out = next - curr;
            a1 = atan2(v_in(2), v_in(1));
            a2 = atan2(v_out(2), v_out(1));
            deltaIfRight = rad2deg(wrapToPi(a2 - a1));
        else
            deltaIfRight = 0; % neutral if not enough history
        end

        % Choose the side with the least absolute turn angle
        if abs(deltaIfLeft) <= abs(deltaIfRight)
            path_l = [path_l; Cones(lastIdx,:)];
            current_l = lastIdx;
        else
            path_r = [path_r; Cones(lastIdx,:)];
            current_r = lastIdx;
        end
        remaining(lastIdx) = false;
        continue;
    end

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
        % Signed turn angle (degrees)
        deltaL = rad2deg(wrapToPi(a2 - a1));
    else
        deltaL = 0; % neutral angle for first step
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
        % Signed turn angle (degrees)
        deltaR = rad2deg(wrapToPi(a2 - a1));
    else
        deltaR = 0; % neutral
    end

    % --- check angles ---
    badL = ~isnan(deltaL) && (deltaL < angle_low || deltaL > angle_high);
    badR = ~isnan(deltaR) && (deltaR < angle_low || deltaR > angle_high);

    %--- if either bad, try swapping ---
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

figure(1);
hold on;

plot(path_r(:,1), path_r(:,2), '-xr')
plot(path_l(:,1), path_l(:,2), '-xb')

plot(Cones(:,1), Cones(:,2), 'xr')


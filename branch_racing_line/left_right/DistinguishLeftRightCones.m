% STEP 1 - Define Parameters
clear all;


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

figure(1);
hold on;

plot(path_r(:,1), path_r(:,2), '-xr')
plot(path_l(:,1), path_l(:,2), '-xb')


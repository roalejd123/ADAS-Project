clc; clear; close all;

% === 맵 및 장애물 정보 불러오기 ===
[start, goal, obstacle_list, space] = map_data();

% === Hybrid A* 파라미터 설정 ===
R = 4;
Vx = 3;
delta_t = 1;
weight = 1.3;

% === 경로 계획 실행 ===
path = hybrid_astar(start, goal, space, obstacle_list, R, Vx, delta_t, weight);

% === 경로 시각화 및 보정 ===
if ~isempty(path)
    % 원시 경로 시각화
    plot(path(:,1), path(:,2), 'c-', 'LineWidth', 1.0);

    % === 급격한 회전 지점 찾기 ===
    yaw = path(:,3);
    dyaw = abs(angdiff(diff(yaw)));
    turn_idx = find(dyaw > deg2rad(30)) + 1;

    % 시작/끝 포함
    turn_idx = [1; turn_idx(:); size(path, 1)];

    % === Reeds-Shepp 보정 적용 ===
    smooth_path = [];
    for i = 1:length(turn_idx)-1
        idx1 = turn_idx(i);
        idx2 = turn_idx(i+1);
        start_seg = path(idx1, :);
        end_seg = path(idx2, :);
        
        [valid, rs_path] = reeds_shepp_connect(start_seg, end_seg, R, obstacle_list, space);
        if valid
            smooth_path = [smooth_path; rs_path];
        else
            smooth_path = [smooth_path; path(idx1:idx2, :)];
        end
    end

    % 보정된 경로 시각화
    plot(smooth_path(:,1), smooth_path(:,2), 'm-', 'LineWidth', 2);
    legend('Boundary', 'Obstacle', 'Start', 'Goal', 'Raw Path', 'Reeds-Shepp Smoothed');
end

% === 아래는 필요한 서브 함수 정의 ===

function path = hybrid_astar(start, goal, space, obstacle_list, R, Vx, delta_time_step, weight)
    res_dijkstra = 1;
    cost_map = run_dijkstra(goal, obstacle_list, space, res_dijkstra);
    start_node = create_node([], start);
    goal_node = create_node([], goal);
    open_list = start_node;
    closed_list = [];

    figure(1); clf; hold on; axis equal;
    xlim([space(1), space(2)]);
    ylim([space(3), space(4)]);
    title('Hybrid A* Path Planning - Realtime Visualization');
    xlabel('X [m]'); ylabel('Y [m]');
    for i = 1:size(obstacle_list, 1)
        viscircles(obstacle_list(i, 1:2), obstacle_list(i, 3), 'Color', 'k', 'LineWidth', 0.8);
    end
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal(1), goal(2), 'mo', 'MarkerSize', 10, 'LineWidth', 2);

    while ~isempty(open_list)
        [~, idx] = min([open_list.f]);
        cur_node = open_list(idx);
        open_list(idx) = [];
        closed_list = [closed_list, cur_node];
        [rs_valid, rs_path] = reeds_shepp_connect(cur_node.position, goal, R, obstacle_list, space);
        if rs_valid
            path = [];
            while ~isempty(cur_node)
                path = [cur_node.position; path];
                cur_node = cur_node.parent;
            end
            path = [path; rs_path];
            return;
        end

        if isSamePosition(cur_node, goal, 5)
            path = [];
            while ~isempty(cur_node)
                path = [cur_node.position; path];
                cur_node = cur_node.parent;
            end
            return;
        end

        actions = get_action(R, Vx, delta_time_step);
        for i = 1:size(actions, 1)
            yaw_rate = actions(i,1);
            dt = actions(i,2);
            new_pos = vehicle_move(cur_node.position, yaw_rate, dt, Vx);
            if isNotInSearchingSpace(new_pos, space), continue; end
            if collision_check(cur_node.position, yaw_rate, dt, obstacle_list, Vx, space), continue; end
            child_node = create_node(cur_node, new_pos);
            skip = false;
            for j = 1:length(closed_list)
                if isSamePosition(closed_list(j), child_node.position, 5) && ...
                        isSameYaw(closed_list(j).heading, child_node.heading, 0.3)
                    skip = true;
                    break;
                end
            end
            if skip, continue; end
            
            reverse_penalty = actions(i,3) < 0;
            yaw_diff = abs(angdiff(cur_node.heading, child_node.heading));
            turn_penalty = 2 * yaw_diff;
            child_node.g = cur_node.g + abs(actions(i,3)) * delta_time_step + turn_penalty + reverse_penalty;
            child_node.h = heuristic(child_node, goal, cost_map, space, res_dijkstra);
            child_node.f = child_node.g + weight * child_node.h;

            skip = false;
            for j = 1:length(open_list)
                if isSamePosition(open_list(j), child_node.position, 5) && child_node.f >= open_list(j).f
                    skip = true; break;
                end
            end
            if skip, continue; end
            open_list = [open_list, child_node];
            plot([cur_node.position(1), child_node.position(1)], [cur_node.position(2), child_node.position(2)], 'c-'); drawnow;
        end
    end
    path = []; disp('No path found.');
end

function h = heuristic(cur_node, goal, cost_map, space, res)
    gx = round((cur_node.position(1) - space(1)) / res) + 1;
    gy = round((cur_node.position(2) - space(3)) / res) + 1;
    [ny, nx] = size(cost_map);
    if gx < 1 || gx > nx || gy < 1 || gy > ny, h = 1e6; return; end
    cost_dijkstra = cost_map(gy, gx);
    dyaw = abs(angdiff(cur_node.heading, goal(3)));
    h = cost_dijkstra + 3 * dyaw;
end

function cost_map = run_dijkstra(goal, obstacle_list, space, res)
    x_range = space(1):res:space(2);
    y_range = space(3):res:space(4);
    nx = length(x_range); ny = length(y_range);
    cost_map = inf(ny, nx); visited = false(ny, nx);
    gx = round((goal(1) - space(1)) / res) + 1;
    gy = round((goal(2) - space(3)) / res) + 1;
    cost_map(gy, gx) = 0; q = [gx, gy];
    while ~isempty(q)
        [~, idx] = min(cost_map(sub2ind(size(cost_map), q(:,2), q(:,1))));
        current = q(idx, :); q(idx, :) = [];
        x = current(1); y = current(2);
        if visited(y, x), continue; end
        visited(y, x) = true;
        for dx = -1:1
            for dy = -1:1
                if dx == 0 && dy == 0, continue; end
                nx_ = x + dx; ny_ = y + dy;
                if nx_ < 1 || ny_ < 1 || nx_ > nx || ny_ > ny, continue; end
                px = space(1) + (nx_-1) * res;
                py = space(3) + (ny_-1) * res;
                in_collision = false;
                for k = 1:size(obstacle_list,1)
                    if hypot(px - obstacle_list(k,1), py - obstacle_list(k,2)) < obstacle_list(k,3)
                        in_collision = true; break;
                    end
                end
                if in_collision, continue; end
                cost = cost_map(y,x) + res * hypot(dx, dy);
                if cost < cost_map(ny_, nx_)
                    cost_map(ny_, nx_) = cost;
                    q = [q; nx_, ny_];
                end
            end
        end
    end
end

function node = create_node(parent, position)
    node.parent = parent;
    node.position = position;
    node.heading = position(3);
    node.g = 0; node.h = 0; node.f = 0;
end

function actions = get_action(R, Vx, delta_time_step)
    yaw_rate = Vx / R;
    actions = [
        yaw_rate, delta_time_step, Vx;
        -yaw_rate, delta_time_step, Vx;
        yaw_rate/2, delta_time_step, Vx;
        -yaw_rate/2, delta_time_step, Vx;
        0.0, delta_time_step, Vx;

        % 후진
        yaw_rate, delta_time_step, -Vx;
        -yaw_rate, delta_time_step, -Vx;
        yaw_rate/2, delta_time_step, -Vx;
        -yaw_rate/2, delta_time_step, -Vx;
        0.0, delta_time_step, -Vx;
    ];
end

function new_pos = vehicle_move(pos, yaw_rate, delta_time, Vx)
    x = pos(1); y = pos(2); yaw = pos(3);
    if abs(yaw_rate) > 1e-3
        R = Vx / yaw_rate;
        cx = x - R * sin(yaw);
        cy = y + R * cos(yaw);
        dtheta = yaw_rate * delta_time;
        x_new = cx + R * sin(yaw + dtheta);
        y_new = cy - R * cos(yaw + dtheta);
        yaw_new = yaw + dtheta;
    else
        x_new = x + Vx * delta_time * cos(yaw);
        y_new = y + Vx * delta_time * sin(yaw);
        yaw_new = yaw;
    end
    yaw_new = mod(yaw_new, 2*pi);
    new_pos = [x_new, y_new, yaw_new];
end

function is_collision = collision_check(pos, yaw_rate, dt, obs_list, Vx, space)
    new_pos = vehicle_move(pos, yaw_rate, dt, Vx);
    x = new_pos(1); y = new_pos(2);
    if x < space(1) || x > space(2) || y < space(3) || y > space(4)
        is_collision = true; return;
    end
    is_collision = false;
    for i = 1:size(obs_list,1)
        if hypot(x - obs_list(i,1), y - obs_list(i,2)) < obs_list(i,3)
            is_collision = true; return;
        end
    end
end

function same = isSamePosition(node, pos, epsilon)
    dx = node.position(1) - pos(1);
    dy = node.position(2) - pos(2);
    same = hypot(dx, dy) < epsilon;
end

function same = isSameYaw(yaw1, yaw2, epsilon)
    same = abs(wrapToPi(yaw1 - yaw2)) < epsilon;
end

function out = isNotInSearchingSpace(pos, space)
    x = pos(1); y = pos(2);
    out = ~(space(1) <= x && x <= space(2) && space(3) <= y && y <= space(4));
end

function [start, goal, obstacle_list, space] = map_data()
    start = [0, -20, 0];
    goal  = [80, -3, 0];
    space = [0, 100, -100, 0];
    car_w = 2.48; car_l = 11.5;
    r = car_w / 2 + 2;
    n_circles = 10;
    interval = car_l / n_circles;
    traffic_info = [40 -12 pi/2; 40 -24 pi/2; 40 -36 pi/2; 40 -48 pi/2; 39 -50 0; 51 -50 0; 63 -50 0];
    obstacle_list = [];
    for i = 1:size(traffic_info, 1)
        x_head = traffic_info(i, 1);
        y_head = traffic_info(i, 2);
        yaw = traffic_info(i, 3);
        for j = 0:n_circles-1
            dx = cos(yaw) * (j * interval);
            dy = sin(yaw) * (j * interval);
            xc = x_head + dx;
            yc = y_head + dy;
            obstacle_list = [obstacle_list; xc, yc, r];
        end
    end
end

function [valid, rs_path] = reeds_shepp_connect(start, goal, R, obstacle_list, space)
    n_points = 50;
    x = linspace(start(1), goal(1), n_points);
    y = linspace(start(2), goal(2), n_points);
    yaw = linspace(start(3), goal(3), n_points);
    rs_path = [x', y', yaw'];
    
    valid = true;
    for i = 1:n_points
        pos = rs_path(i, :);
        if isNotInSearchingSpace(pos, space) || ...
           collision_check(pos, 0, 0, obstacle_list, 0.01, space)
            valid = false;
            return;
        end
    end
end

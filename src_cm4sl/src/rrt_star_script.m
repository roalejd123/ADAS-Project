
function rrt_star_rs()
    % 🚗 사용자 지정 환경 설정
    map_boundary = [0, 0, 100, 0, 100, -100, 0, -100];
     traffic_info = [
        % 🔴 왼쪽 세로 주차열 (회전 주차)
        20, -20, pi/2;
        30, -20, pi/2;
        40, -20, pi/2;

        % 🔴 가운데 가로 주차열
        40, -50, 0;
        60, -50, 0;
        80, -50, 0;

        % 🔴 오른쪽 세로 주차열
        70, -20, pi/2;
        80, -20, pi/2;

        % 🔴 혼잡 영역 추가
        50, -75, pi/2;
        60, -75, pi/2;

        % 🔴 자유 공간 사이 장애물 (불규칙)
        
        65, -30, 0;
        65, -30, 0;
        ];
    traffic_size = [2.48, 11.5];
    start = [0, -20, 0];  % 초기 yaw 포함
    goal = [55, -50, pi/2]; % 목표 yaw 포함
    space = [0, 100, -100, 0];

    config.eta = 1;
    config.gamma = 5;
    config.goal_sample_rate = 0.5;
    config.goal_range = 2.0;
    config.min_turning_radius = 5;  % 최소 회전 반경
    max_iter = 3000;

    % === 장애물 원형 근사 ===
    obstacles = [];
    r_buffer = 1.0;
    w = traffic_size(1); l = traffic_size(2);
    for i = 1:size(traffic_info, 1)
        yaw = traffic_info(i,3);
        rear_offset = (l/2) * [-cos(yaw), -sin(yaw)];
        center = traffic_info(i,1:2) - rear_offset;
        obstacles(end+1,:) = [center, max(w, l)/2 + r_buffer];
    end

    % 초기 노드 설정
    nodes(1).id = 1;
    nodes(1).pos = start;
    nodes(1).cost = 0;
    nodes(1).parent = 0;

    node_id = 2;
    goal_id = -1;

    figure; hold on; axis equal; grid on;
    plot_obstacles(obstacles);
    plot(start(1), start(2), 'go', 'MarkerFaceColor', 'g');
    plot(goal(1), goal(2), 'rx', 'MarkerSize', 10);

    for i = 1:max_iter
        if rand < config.goal_sample_rate
            sample = goal;
        else
            sample = [rand_range(space(1), space(2)), rand_range(space(3), space(4)), rand_range(-pi, pi)];
        end

        nearest = get_nearest_node(nodes, sample);
        new_pos = reeds_shepp_steer(nearest.pos, sample, config.min_turning_radius);

        if ~is_collision_free(nearest.pos, new_pos, obstacles)
            continue;
        end

        nodes(node_id).id = node_id;
        nodes(node_id).pos = new_pos;
        nodes(node_id).cost = nearest.cost + reeds_shepp_path_length(nearest.pos, new_pos);
        nodes(node_id).parent = nearest.id;

        if norm(new_pos(1:2) - goal(1:2)) < config.goal_range
            goal_id = node_id;
            break;
        end

        plot([nearest.pos(1), new_pos(1)], [nearest.pos(2), new_pos(2)], 'b-');
        node_id = node_id + 1;
    end

    if goal_id > 0
        path = [];
        current = goal_id;
        while current > 0
            path = [nodes(current).pos; path];
            current = nodes(current).parent;
        end

        % 1단계: RRT*-Smart로 최적 경로 탐색
        optimized_path = rrt_star_smart(path, obstacles, config);
    
        % 2단계: 클로소이드 스무딩
        smooth_path = path_clothoid_smoothing(optimized_path);

        % 최종 경로 시각화
        plot(smooth_path(:,1), smooth_path(:,2), 'r-', 'LineWidth', 2);
        %plot(optimized_path(:,1), optimized_path(:,2), 'r-', 'LineWidth', 2);
        title('RRT* with Smart Path Optimization and Clothoid Smoothing');
    else
        title('경로를 찾지 못했습니다.');
    end
    detect_reverse_path(smooth_path);
end

function val = rand_range(a, b)
    val = a + (b - a) * rand;
end

function plot_obstacles(obs)
    theta = linspace(0, 2*pi, 100);
    for i = 1:size(obs, 1)
        x = obs(i,1) + obs(i,3)*cos(theta);
        y = obs(i,2) + obs(i,3)*sin(theta);
        fill(x, y, 'r', 'FaceAlpha', 0.2);
    end
end

function nearest = get_nearest_node(nodes, sample)
    d = arrayfun(@(n) norm(n.pos - sample), nodes);
    [~, idx] = min(d);
    nearest = nodes(idx);
end

function near_ids = get_near_nodes(nodes, pos, config)
    n = length(nodes);
    r = min(config.gamma * sqrt(log(n)/n), config.eta);
    near_ids = find(arrayfun(@(n) norm(n.pos - pos) <= r, nodes));
end

function new = steer(from, to, eta)
    d = norm(to - from);
    if d < eta
        new = to;
    else
        new = from + (to - from) / d * eta;
    end
end

function flag = is_collision_free(p1, p2, obstacles)
    steps = ceil(norm(p2(1:2) - p1(1:2)) / 0.2);  % 거리 계산 시 yaw 제외
    for i = 0:steps
        pt = p1(1:2) + (p2(1:2) - p1(1:2)) * i / steps;  % 위치 정보만 사용
        for j = 1:size(obstacles, 1)
            dist = norm(pt - obstacles(j,1:2));  % 거리 계산
            if dist <= obstacles(j,3)
                flag = false;
                return;
            end
        end
    end
    flag = true;
end

function new_pos = reeds_shepp_steer(from, to, min_turning_radius)
    dx = to(1) - from(1);
    dy = to(2) - from(2);
    dtheta = to(3) - from(3);
    dist = sqrt(dx^2 + dy^2);

    % 전진과 후진 고려
    if rand > 0.5
        move_direction = 1;  % 전진
    else
        move_direction = -1; % 후진
    end

    % 회전 반경을 고려한 이동
    max_angle = atan2(dy, dx);
    move_distance = min(dist, min_turning_radius);
    new_x = from(1) + move_direction * cos(max_angle) * move_distance;
    new_y = from(2) + move_direction * sin(max_angle) * move_distance;
    new_yaw = from(3) + dtheta;

    new_pos = [new_x, new_y, new_yaw];
end

function length = reeds_shepp_path_length(from, to)
    dx = to(1) - from(1);
    dy = to(2) - from(2);
    dtheta = abs(to(3) - from(3));
    length = sqrt(dx^2 + dy^2) + dtheta;
end

function detect_reverse_path(path)
    figure; hold on; axis equal; grid on;
    plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2);
    title('Reverse Path Detection');
    
    % 후진 구간 탐지
    for i = 2:size(path, 1) - 1
        prev = path(i - 1, 1:2);
        curr = path(i, 1:2);
        next = path(i + 1, 1:2);

        % 방향 벡터 계산
        v1 = curr - prev;
        v2 = next - curr;

        % 방향 벡터의 내적 계산
        dot_product = dot(v1, v2);

        % 후진 여부 판단: 내적이 음수이면 후진
        if dot_product < 0
            plot(curr(1), curr(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2); % 후진 구간 표시
        end
    end
    xlabel('X'); ylabel('Y');
end

function smooth_path = path_clothoid_smoothing(path)
    if size(path, 1) < 3
        smooth_path = path;
        return;
    end

    % 직선 구간 탐지 및 고정
    fixed_points = false(size(path, 1), 1);
    for i = 2:size(path, 1) - 1
        prev = path(i - 1, 1:2);
        curr = path(i, 1:2);
        next = path(i + 1, 1:2);
        
        % 세 점이 직선상에 있는지 확인
        angle1 = atan2(curr(2) - prev(2), curr(1) - prev(1));
        angle2 = atan2(next(2) - curr(2), next(1) - curr(1));
        if abs(angle1 - angle2) < deg2rad(5)
            fixed_points(i) = true;
        end
    end

    % 곡선 보간 (클로소이드) - 직선 구간 제외
    x = path(:, 1);
    y = path(:, 2);
    s = cumsum([0; sqrt(diff(x).^2 + diff(y).^2)]);  % 호 길이 계산
    ts = linspace(0, s(end), 5 * numel(s));

    % 곡률 제어를 위한 PCHIP 보간
    x_smooth = pchip(s(~fixed_points), x(~fixed_points), ts);
    y_smooth = pchip(s(~fixed_points), y(~fixed_points), ts);

    % 곡률 제한을 통한 부드러운 연결
    max_curvature = 0.05;  % 곡률 최대 값 조정
    dx = gradient(x_smooth);
    dy = gradient(y_smooth);
    ddx = gradient(dx);
    ddy = gradient(dy);
    curvature = abs((dx .* ddy - dy .* ddx) ./ (dx.^2 + dy.^2).^(3/2));

    % 곡률이 과도하면 완만하게 보정
    for k = 1:length(curvature)
        if curvature(k) > max_curvature
            factor = max_curvature / curvature(k);
            x_smooth(k) = x_smooth(k-1) + factor * (x_smooth(k) - x_smooth(k-1));
            y_smooth(k) = y_smooth(k-1) + factor * (y_smooth(k) - y_smooth(k-1));
        end
    end

    smooth_path = [x_smooth', y_smooth'];
end

function optimized_path = rrt_star_smart(path, obstacles, config)
    optimized_path = path;
    i = 1;
    while i < size(optimized_path, 1) - 1
        for j = size(optimized_path, 1):-1:i+2
            % 직선 경로로 바로 연결 가능한지 확인
            if is_collision_free(optimized_path(i, 1:2), optimized_path(j, 1:2), obstacles)
                % 경로를 직선으로 연결하여 최적화
                optimized_path = [optimized_path(1:i, :); optimized_path(j:end, :)];
                break;
            end
        end
        i = i + 1;
    end
end
function main_rrt_rs()
    clc; clear; close all;

    [start, goal, obstacle_list, space] = map_data();
    max_iter = 3000; step_size = 1; goal_sample_rate = 0.4; radius = 10; R = 7;
    
    nodes(1).pos = start;
    nodes(1).parent = 0;
    nodes(1).cost = 0;

    figure(1); clf; hold on; axis equal;
    xlim([space(1), space(2)]); ylim([space(3), space(4)]);
    title('Informed RRT* with RS'); xlabel('X [m]'); ylabel('Y [m]');
    for i = 1:size(obstacle_list,1)
        viscircles(obstacle_list(i,1:2), obstacle_list(i,3), 'Color', 'k');
    end
    plot(start(1), start(2), 'go', 'MarkerSize', 10);
    plot(goal(1), goal(2), 'mo', 'MarkerSize', 10);

    c_best = inf; best_goal_node = -1; rs_to_goal = [];

    for iter = 1:max_iter
        if rand < goal_sample_rate
            sample = goal;
        elseif isfinite(c_best)
            sample = informed_sample(start, goal, c_best, space);
        else
            sample = [rand_range(space(1), space(2)), rand_range(space(3), space(4)), rand * 2 * pi];
        end

        dists = vecnorm(cell2mat(arrayfun(@(n) n.pos(1:2)', nodes, 'UniformOutput', false)) - sample(1:2)', 2, 1);
        [~, nearest_id] = min(dists);
        nearest_node = nodes(nearest_id);

        theta = atan2(sample(2) - nearest_node.pos(2), sample(1) - nearest_node.pos(1));
        new_pos = nearest_node.pos(1:2) + step_size * [cos(theta), sin(theta)];
        new_pos = [new_pos, theta];

        if ~collision_check_line(nearest_node.pos, new_pos, obstacle_list, space)
            new_node.pos = new_pos;
            new_node.cost = nearest_node.cost + norm(new_pos(1:2) - nearest_node.pos(1:2));
            new_node.parent = nearest_id;

            near_ids = find(vecnorm(cell2mat(arrayfun(@(n) n.pos(1:2)', nodes, 'UniformOutput', false)) - new_node.pos(1:2)', 2, 1) < radius);
            min_cost = new_node.cost;
            for nid = near_ids
                temp_cost = nodes(nid).cost + norm(nodes(nid).pos(1:2) - new_node.pos(1:2));
                if temp_cost < min_cost && ~collision_check_line(nodes(nid).pos, new_node.pos, obstacle_list, space)
                    new_node.cost = temp_cost;
                    new_node.parent = nid;
                    min_cost = temp_cost;
                end
            end

            nodes(end+1) = new_node;
            plot([nodes(new_node.parent).pos(1), new_node.pos(1)], [nodes(new_node.parent).pos(2), new_node.pos(2)], 'c-'); drawnow;

            if norm(new_node.pos(1:2) - goal(1:2)) < 25
                [valid, rs_path] = reeds_shepp_connect(new_node.pos, goal, R, obstacle_list, space);
                if valid
                    c_rs = new_node.cost + sum(vecnorm(diff(rs_path(:,1:2)), 2, 2));
                    if c_rs < c_best
                        c_best = c_rs;
                        best_goal_node = length(nodes);
                        rs_to_goal = rs_path;
                        break;
                    end
                end
            end
        end
    end

    if best_goal_node > 0
        % 전체 경로 추적
        rrt_path = [];
        current = best_goal_node;
        while current > 0
            rrt_path = [nodes(current).pos; rrt_path];
            current = nodes(current).parent;
        end
        smooth_path = path_clothoid_smoothing(rrt_path);
        % RS 보정 경로는 이미 rs_to_goal에 있음
        rs_path = rs_to_goal;

        % 전체 경로 = RRT* + RS 보정
        full_path = [smooth_path; rs_path];
        assignin('base', 'full_path', full_path);
        
        % === 시각화 ===
        %plot(rrt_path(:,1), rrt_path(:,2), 'b--', 'LineWidth', 1.5);      % RRT* 경로
        %plot(rs_path(:,1), rs_path(:,2), 'm-', 'LineWidth', 2.5);         % RS 보정 경로 (강조)
        plot(full_path(:,1), full_path(:,2), 'r-', 'LineWidth', 2.5);
        % 목표점 표시
        plot(goal(1), goal(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
    else
        disp('Path not found.');
    end
    % === 서브 함수들 ===
    function s = rand_range(a, b)
        s = a + (b - a) * rand;
    end

    function sample = informed_sample(start, goal, c_max, space)
        c_min = norm(goal(1:2) - start(1:2));
        a = c_max / 2;
        b = sqrt(c_max^2 - c_min^2) / 2;
        theta = atan2(goal(2) - start(2), goal(1) - start(1));
        C = [cos(theta), -sin(theta); sin(theta), cos(theta)];
        while true
            x = randn * a; y = randn * b;
            if x^2 / a^2 + y^2 / b^2 <= 1, break; end
        end
        pos = C * [x; y] + (start(1:2)' + goal(1:2)') / 2;
        sample = [pos', rand * 2 * pi];
    end

    function collision = collision_check_line(p1, p2, obs_list, space)
        n = 50;
        xs = linspace(p1(1), p2(1), n);
        ys = linspace(p1(2), p2(2), n);
        collision = false;
        for i = 1:n
            if xs(i) < space(1) || xs(i) > space(2) || ys(i) < space(3) || ys(i) > space(4)
                collision = true; return;
            end
            for j = 1:size(obs_list, 1)
                margin = 1.5;
                if hypot(xs(i) - obs_list(j,1), ys(i) - obs_list(j,2)) < (obs_list(j,3) + margin)
                    collision = true; return;
                end
            end
        end
    end
end

function [start, goal, obstacle_list, space] = map_data()
    start = [0, -20, 0];
    goal  = [80, -3, pi/2];
    space = [0, 100, -100, 0];
    car_w = 2.48; car_l = 11.5;
    r = car_w / 2 + 2;
    n_circles = 10;
    interval = car_l / n_circles;
    traffic_info = [40 -12 pi/2; 40 -24 pi/2; 40 -36 pi/2; 40 -48 pi/2; 65 -36 pi/2; 65 -48 pi/2; 39 -50 0; 51 -50 0; 63 -50 0];
    % traffic_info = [
    %     10, -35, pi/2;
    %     45, -55, pi/2;
    %     65, -75, pi/2;
    %     70, -35, pi/2;
    %     % 1열 왼쪽 세로 주차열
    %     20, -35, pi/2;
    %     20, -55, pi/2;
    %     20, -75, pi/2;
    %     20, -95, pi/2;
    % 
    %     % 2열 중간 세로 주차열
    %     40, -35, pi/2;
    %     40, -55, pi/2;
    %     40, -75, pi/2;
    %     40, -95, pi/2;
    % 
    %     % 3열 오른쪽 세로 주차열
    %     60, -35, pi/2;
    %     60, -55, pi/2;
    %     60, -75, pi/2;
    %     60, -95, pi/2;
    % 
    %     % 오른쪽 세로형 주차열 (끝에 추가)
    %     80, -35, pi/2;
    %     80, -55, pi/2;
    %     80, -75, pi/2;
    %     80, -95, pi/2;
    % ];
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


function smooth_path = path_clothoid_smoothing(path)
    if size(path, 1) < 3
        smooth_path = path;
        return;
    end

    % === 파라미터 ===
    max_curvature = 0.1;   % 허용 곡률 최대값
    density_factor = 100;    % 포인트 밀도 증가 계수 (기존보다 몇 배 늘릴지)

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
    yaw = path(:, 3);
    s = cumsum([0; sqrt(diff(x).^2 + diff(y).^2)]);  % 호 길이 계산
    ts = linspace(0, s(end), 5 * numel(s));

    % 곡률 제어를 위한 PCHIP 보간
    x_smooth = pchip(s(~fixed_points), x(~fixed_points), ts);
    y_smooth = pchip(s(~fixed_points), y(~fixed_points), ts);
    
    yaw_unwrap = unwrap(yaw);
    yaw_smooth = pchip(s(~fixed_points), yaw_unwrap(~fixed_points), ts);

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

    smooth_path = [x_smooth', y_smooth', wrapToPi(yaw_smooth')];
end

function [valid, rs_path] = reeds_shepp_connect(start, goal, R, obstacle_list, space)
    param.step = 0.5;
    param.max_curv = 1 / R;
    [x_list, y_list, yaw_list] = generation(start, goal, param);
    rs_path = [x_list(:), y_list(:), yaw_list(:)];
    valid = true;
    for i = 1:length(rs_path)
        pos = rs_path(i, :);
        if collision_check_point(pos, obstacle_list, space)
            valid = false;
            return;
        end
    end
end

function is_collision = collision_check_point(pos, obstacle_list, space)
    x = pos(1); y = pos(2);
    if x < space(1) || x > space(2) || y < space(3) || y > space(4)
        is_collision = true; return;
    end
    is_collision = false;
    for i = 1:size(obstacle_list, 1)
        if hypot(x - obstacle_list(i,1), y - obstacle_list(i,2)) < obstacle_list(i,3) + 1.5
            is_collision = true; return;
        end
    end
end

function [x_list, y_list, yaw_list] = generation(start_pose, goal_pose, param)
    sx = start_pose(1); sy = start_pose(2); syaw = start_pose(3);
    gx = goal_pose(1); gy = goal_pose(2); gyaw = goal_pose(3);
    
    dx = gx - sx; dy = gy - sy; dyaw = gyaw - syaw;
    x = (cos(syaw) * dx + sin(syaw) * dy) * param.max_curv;
    y = (-sin(syaw) * dx + cos(syaw) * dy) * param.max_curv;

    planners = ["SCS", "CCC", "CSC", "CCCC", "CCSC", "CCSCC"];
    best_cost = inf;
    best_path = [];

    for i = 1:length(planners)
        planner = str2func(planners(i));
        paths = planner(x, y, dyaw);
        for j = 1:length(paths)
            if paths(j).len < best_cost
                best_path = paths(j);
                best_cost = paths(j).len;
            end
        end
    end

    points_num = floor(best_cost / param.step) + length(best_path.segs) + 3;
    x_list_ = zeros(points_num); y_list_ = zeros(points_num); yaw_list_ = zeros(points_num);
    i = 1;
    for j = 1:length(best_path.segs)
        m = best_path.ctypes(j);
        seg_length = best_path.segs(j);
        d_length = param.step * sign(seg_length);
        x = x_list_(i); y = y_list_(i); yaw = yaw_list_(i);
        l = d_length;
        while abs(l) <= abs(seg_length)
            i = i + 1;
            new_pt = interpolate(m, l, [x, y, yaw], param);
            x_list_(i) = new_pt(1); y_list_(i) = new_pt(2); yaw_list_(i) = new_pt(3);
            l = l + d_length;
        end
        i = i + 1;
        new_pt = interpolate(m, seg_length, [x, y, yaw], param);
        x_list_(i) = new_pt(1); y_list_(i) = new_pt(2); yaw_list_(i) = new_pt(3);
    end

    x_list_ = x_list_(1:i); y_list_ = y_list_(1:i); yaw_list_ = yaw_list_(1:i);
    x_list = zeros(size(x_list_)); y_list = zeros(size(y_list_)); yaw_list = zeros(size(yaw_list_));
    for k = 1:length(x_list_)
        x_list(k) = cos(-syaw) * x_list_(k) + sin(-syaw) * y_list_(k) + sx;
        y_list(k) = -sin(-syaw) * x_list_(k) + cos(-syaw) * y_list_(k) + sy;
        yaw_list(k) = M(yaw_list_(k) + syaw);
    end
end

%%
function [r, theta] = R(x, y)
    r = hypot(x, y);
    theta = atan2(y, x);
end

function theta_truncate = M(theta)
    while theta > pi
        theta = theta - 2.0 * pi;
    end
    while theta < -pi
        theta = theta + 2.0 * pi;
    end
    theta_truncate = theta;
end

function [tau, omega] = calTauOmega(u, v, xi, eta, phi)
    delta = M(u - v);
    A = sin(u) - sin(delta);
    B = cos(u) - cos(delta) - 1.0;

    t1 = atan2(eta * A - xi * B, xi * A + eta * B);
    t2 = 2.0 * (cos(delta) - cos(v) - cos(u)) + 3.0;

    if t2 < 0
        tau = M(t1 + math.pi);
    else
        tau = M(t1);
    end
    omega = M(tau - u + v - phi);
end

function segs = SLS(x, y, phi)
    % Straight-Left-Straight generation mode.
    segs = [];
    
    phi = M(phi);
    if y > 0.0 && phi > 0.0 && phi < pi * 0.99
        xd = -y / tan(phi) + x;
        t = xd - tan(phi / 2.0);
        u = phi;
        v = hypot(x - xd, y) - tan(phi / 2.0);
        segs = [t, u, v];
    elseif y < 0.0 && phi > 0.0 && phi < pi * 0.99
        xd = -y / tan(phi) + x;
        t = xd - tan(phi / 2.0);
        u = phi;
        v = -hypot(x - xd, y) - tan(phi / 2.0);
        segs = [t, u, v];
    end
end

function segs = LRL(x, y, phi)
    % Left-Right-Left generation mode. (L+R-L-)
    segs = [];
    [r, theta] = R(x - sin(phi), y - 1.0 + cos(phi));
    if r <= 4.0
        u = -2.0 * asin(0.25 * r);
        t = M(theta + 0.5 * u + pi);
        v = M(phi - t + u);
        if t >= 0.0 && u <= 0.0
            segs = [t, u, v];
        end
    end
end

function segs = LSL(x, y, phi)
    % Left-Straight-Left generation mode. (L+S+L+)
    segs = [];
    
    [u, t] = R(x - sin(phi), y - 1.0 + cos(phi));
    if t >= 0.0
        v = M(phi - t);
        if v >= 0.0
            segs = [t, u, v];
        end
    end
end

function segs = LSR(x, y, phi)
    % Left-Straight-Right generation mode. (L+S+R+)
    segs = [];
    
    [r, theta] = R(x + sin(phi), y - 1.0 - cos(phi));
    r = r * r;
    if r >= 4.0
        u = sqrt(r - 4.0);
        t = M(theta + atan2(2.0, u));
        v = M(t - phi);
        if t >= 0.0 && v >= 0.0
            segs = [t, u, v];
        end
    end
end

function segs = LRLRn(x, y, phi)
    % Left-Right(beta)-Left(beta)-Right generation mode. (L+R+L-R-)
    segs = [];
    
    xi = x + sin(phi);
    eta = y - 1.0 - cos(phi);
    rho = 0.25 * (2.0 + hypot(xi, eta));
    if rho <= 1.0
        u = acos(rho);
        [t, v] = calTauOmega(u, -u, xi, eta, phi);
        if t >= 0.0 && v <= 0.0
            segs = [t, u, v];
        end
    end
end

function segs = LRLRp(x, y, phi)
    % Left-Right(beta)-Left(beta)-Right generation mode. (L+R-L-R+)
    segs = [];
    
    xi = x + sin(phi);
    eta = y - 1.0 - cos(phi);
    rho = (20.0 - xi * xi - eta * eta) / 16.0;
    if 0.0 <= rho && rho <= 1.0
        u = -acos(rho);
        if u >= -0.5 * pi
            [t, v] = calTauOmega(u, u, xi, eta, phi);
            if t >= 0.0 && v >= 0.0
                segs = [t, u, v];
            end
        end
    end
end

function segs = LRSR(x, y, phi)
    % Left-Right(pi/2)-Straight-Right generation mode. (L+R-S-R-)
    segs = [];
    
    xi = x + sin(phi);
    eta = y - 1.0 - cos(phi);
    [rho, theta] = R(-eta, xi);

    if rho >= 2.0
        t = theta;
        u = 2.0 - rho;
        v = M(t + 0.5 * pi - phi);
        if t >= 0.0 && u <= 0.0 && v <= 0.0
            segs = [t, u, v];
        end
    end
end

function segs = LRSL(x, y, phi)
    % Left-Right(pi/2)-Straight-Left generation mode. (L+R-S-L-)
    segs = [];
    
    xi = x - sin(phi);
    eta = y - 1.0 + cos(phi);
    [rho, theta] = R(xi, eta);
    if rho >= 2.0
        r = sqrt(rho * rho - 4.0);
        u = 2.0 - r;
        t = M(theta + atan2(r, -2.0));
        v = M(phi - 0.5 * pi - t);
        if t >= 0.0 && u <= 0.0 && v <= 0.0
            segs = [t, u, v];
        end
    end
end

function segs = LRSLR(x, y, phi)
    % Left-Right(pi/2)-Straight-Left(pi/2)-Right generation mode. (L+R-S-L-R+)
    segs = [];
    
    xi = x + sin(phi);
    eta = y - 1.0 - cos(phi);
    [r, ~] = R(xi, eta);
    if r >= 2.0
        u = 4.0 - sqrt(r * r - 4.0);
        if u <= 0.0
            t = M(atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
            v = M(t - phi);
            if t >= 0.0 && v >= 0.0
                segs = [t, u, v];
            end
        end
    end
end

function path = RSPath(segs, ctypes)
    path.segs = segs;
    path.ctypes = ctypes;
    path.len = sum(abs(segs));
end
    
function paths = SCS(x, y, phi)
    paths = struct('segs', {}, 'ctypes', {}, 'len', {});

    segs = SLS(x, y, phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["S", "L", "S"]);
    end
        
    segs = SLS(x, -y, -phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["S", "R", "S"]);
    end
end

function paths = CCC(x, y, phi)
    paths = struct('segs', {}, 'ctypes', {}, 'len', {});

    % L+R-L-
    segs = LRL(x, y, phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["L", "R", "L"]);
    end

    % timefilp: L-R+L+
    segs = LRL(-x, y, -phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(-segs, ["L", "R", "L"]);
    end

    % reflect: R+L-R-
    segs = LRL(x, -y, -phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["R", "L", "R"]);
    end

    % timeflip + reflect: R-L+R+
    segs = LRL(-x, -y, phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(-segs, ["R", "L", "R"]);
    end

    % backwards
    xb = x * cos(phi) + y * sin(phi);
    yb = x * sin(phi) - y * cos(phi);

    % backwards: L-R-L+
    segs = LRL(xb, yb, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([v, u, t], ["L", "R", "L"]);
    end

    % backwards + timefilp: L+R+L-
    segs = LRL(-xb, yb, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-v, -u, -t], ["L", "R", "L"]);
    end

    % backwards + reflect: R-L-R+
    segs = LRL(xb, -yb, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([v, u, t], ["R", "L", "R"]);
    end

    % backwards + timeflip + reflect: R+L+R-
    segs = LRL(-xb, -yb, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-v, -u, -t], ["R", "L", "R"]);
    end
end

function paths = CSC(x, y, phi)
    paths = struct('segs', {}, 'ctypes', {}, 'len', {});

    % L+S+L+
    segs = LSL(x, y, phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["L", "S", "L"]);
    end

    % timefilp: L-S-L-
    segs = LSL(-x, y, -phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(-segs, ["L", "S", "L"]);
    end

    % reflect: R+S+R+
    segs = LSL(x, -y, -phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["R", "S", "R"]);
    end

    % timeflip + reflect: R-S-R-
    segs = LSL(-x, -y, phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(-segs, ["R", "S", "R"]);
    end

    % L+S+R+
    segs = LSR(x, y, phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["L", "S", "R"]);
    end

    % timefilp: L-S-R-
    segs = LSR(-x, y, -phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(-segs, ["L", "S", "R"]);
    end

    % reflect: R+S+L+
    segs = LSR(x, -y, -phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(segs, ["R", "S", "L"]);
    end

    % timeflip + reflect: R+S+l-
    segs = LSR(-x, -y, phi);
    if ~isempty(segs)
        paths(end + 1) = RSPath(-segs, ["R", "S", "L"]);
    end
end

function paths = CCCC(x, y, phi)
    paths = struct('segs', {}, 'ctypes', {}, 'len', {});

    % L+R+L-R-
    segs = LRLRn(x, y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, u, -u, v], ["L", "R", "L", "R"]);
    end

    % timefilp: L-R-L+R+
    segs = LRLRn(-x, y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, -u, u, -v], ["L", "R", "L", "R"]);
    end

    % reflect: R+L+R-L-
    segs = LRLRn(x, -y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, u, -u, v], ["R", "L", "R", "L"]);
    end

    % timeflip + reflect: R-L-R+L+
    segs = LRLRn(-x, -y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, -u, u, -v], ["R", "L", "R", "L"]);
    end

    % L+R-L-R+
    segs = LRLRp(x, y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, u, u, v], ["L", "R", "L", "R"]);
    end

    % timefilp: L-R+L+R-
    segs = LRLRp(-x, y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, -u, -u, -v], ["L", "R", "L", "R"]);
    end

    % reflect: R+L-R-L+
    segs = LRLRp(x, -y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, u, u, v], ["R", "L", "R", "L"]);
    end

    % timeflip + reflect: R-L+R+L-
    segs = LRLRp(-x, -y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, -u, -u, -v], ["R", "L", "R", "L"]);
    end
end

function paths = CCSC(x, y, phi)
    paths = struct('segs', {}, 'ctypes', {}, 'len', {});

    % L+R-(pi/2)S-L-
    segs = LRSL(x, y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, -0.5 * pi, u, v], ["L", "R", "S", "L"]);
    end

    % timefilp: L-R+(pi/2)S+L+
    segs = LRSL(-x, y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, 0.5 * pi, -u, -v], ["L", "R", "S", "L"]);
    end

    % reflect: R+L-(pi/2)S-R-
    segs = LRSL(x, -y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, -0.5 * pi, u, v], ["R", "L", "S", "R"]);
    end

    % timeflip + reflect: R-L+(pi/2)S+R+
    segs = LRSL(-x, -y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, 0.5 * pi, -u, -v], ["R", "L", "S", "R"]);
    end

    % L+R-(pi/2)S-R-
    segs = LRSR(x, y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, -0.5 * pi, u, v], ["L", "R", "S", "R"]);
    end

    % timefilp: L-R+(pi/2)S+R+
    segs = LRSR(-x, y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, 0.5 * pi, -u, -v], ["L", "R", "S", "R"]);
    end

    % reflect: R+L-(pi/2)S-L-
    segs = LRSR(x, -y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, -0.5 * pi, u, v], ["R", "L", "S", "L"]);
    end

    % timeflip + reflect: R-L+(pi/2)S+L+
    segs = LRSR(-x, -y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, 0.5 * pi, -u, -v], ["R", "L", "S", "L"]);
    end

    % backwards
    xb = x * cos(phi) + y * sin(phi);
    yb = x * sin(phi) - y * cos(phi);

    % backwards: L-S-R-(pi/2)L+
    segs = LRSL(xb, yb, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([v, u, -0.5 * pi, t], ["L", "S", "R", "L"]);
    end

    % backwards + timefilp: L+S+R+(pi/2)L-
    segs = LRSL(-xb, yb, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-v, -u, 0.5 * pi, -t], ["L", "S", "R", "L"]);
    end

    % backwards + reflect: R-S-L-(pi/2)R+
    segs = LRSL(xb, -yb, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([v, u, -0.5 * pi, t], ["R", "S", "L", "R"]);
    end

    % backwards + timefilp + reflect: R+S+L+(pi/2)R-
    segs = LRSL(-xb, -yb, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-v, -u, 0.5 * pi, -t], ["R", "S", "L", "R"]);
    end

    % backwards: R-S-R-(pi/2)L+
    segs = LRSR(xb, yb, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([v, u, -0.5 * pi, t], ["R", "S", "R", "L"]);
    end

    % backwards + timefilp: R+S+R+(pi/2)L-
    segs = LRSR(-xb, yb, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-v, -u, 0.5 * pi, -t], ["R", "S", "R", "L"]);
    end

    % backwards + reflect: L-S-L-(pi/2)R+
    segs = LRSR(xb, -yb, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([v, u, -0.5 * pi, t], ["L", "S", "L", "R"]);
    end

    % backwards + timefilp + reflect: L+S+L+(pi/2)R-
    segs = LRSR(-xb, -yb, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-v, -u, 0.5 * pi, -t], ["L", "S", "L", "R"]);
    end
end

function paths = CCSCC(x, y, phi)
    paths = struct('segs', {}, 'ctypes', {}, 'len', {});

    % L+R-(pi/2)S-L-(pi/2)R+
    segs = LRSLR(x, y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, -0.5 * pi, u, -0.5 * pi, v], ["L", "R", "S", "L", "R"]);
    end

    % timefilp: L-R+(pi/2)S+L+(pi/2)R-
    segs = LRSLR(-x, y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, 0.5 * pi, -u, 0.5 * pi, -v], ["L", "R", "S", "L", "R"]);
    end

    % reflect: R+L-(pi/2)S-R-(pi/2)L+
    segs = LRSLR(x, -y, -phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([t, -0.5 * pi, u, -0.5 * pi, v], ["R", "L", "S", "R", "L"]);
    end

    % timefilp + reflect: R-L+(pi/2)S+R+(pi/2)L-
    segs = LRSLR(-x, -y, phi);
    if ~isempty(segs)
        t = segs(1); u = segs(2); v = segs(3);
        paths(end + 1) = RSPath([-t, 0.5 * pi, -u, 0.5 * pi, -v], ["R", "L", "S", "R", "L"]);
    end
end

function new_pt = interpolate(mode, length, init_pose, param)
    x = init_pose(1); y = init_pose(2); yaw = init_pose(3);

    if mode == "S"
        new_x   = x + length / param.max_curv * cos(yaw);
        new_y   = y + length / param.max_curv * sin(yaw);
        new_yaw = yaw;
    elseif mode == "L"
        new_x   = x + (sin(yaw + length) - sin(yaw)) / param.max_curv;
        new_y   = y - (cos(yaw + length) - cos(yaw)) / param.max_curv;
        new_yaw = yaw + length;
    elseif mode == "R"
        new_x   = x - (sin(yaw - length) - sin(yaw)) / param.max_curv;
        new_y   = y + (cos(yaw - length) - cos(yaw)) / param.max_curv;
        new_yaw = yaw - length;
    else
        new_x = 0; new_y = 0; new_yaw = 0;
    end
    new_pt = [new_x, new_y, new_yaw];
end
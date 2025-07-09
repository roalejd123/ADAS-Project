% === Simulink-Compatible main_rrt_rs.m (with smoothing & RS connection) ===
full_path = main_rrt_rs(s_point, f_point, obs_list); %#codegen


function full_path = main_rrt_rs(start, goal, obstacle_list) %#codegen
    disp("MAP making start")
    space = [1, 47.5, -44.9, -4.1];
    max_iter = 6000; step_size = 1; goal_sample_rate = 0.4; radius = 10; R = 7;
    MAX_NODES = 3000;
    default_node = struct('pos', zeros(1,3), 'parent', int32(0), 'cost', Inf);
    nodes = repmat(default_node, 1, MAX_NODES);
    nodes(1).pos = start(:)';
    nodes(1).parent = int32(0);
    nodes(1).cost = 0.0;
    node_count = int32(1);
    goal = reshape(goal, 1, 3);
    start = reshape(start, 1, 3);

    c_best = inf; best_goal_node = int32(-1);
    rs_to_goal = NaN(200,3); rs_count = int32(0);

    for iter = 1:max_iter
        if rand < goal_sample_rate
            sample = goal;
        elseif isfinite(c_best)
            sample = informed_sample(start, goal, c_best, space);
        else
            sample = [ ...
                rand_range(space(1), space(2)), ...
                rand_range(space(3), space(4)), ...
                rand * 2 * pi ...
                ];
        end
        sample = reshape(sample, 1, 3);  % ✅ 안전성 확보
        min_dist = inf;
        nearest_id = int32(1);
        for i = 1:node_count
            d = norm(nodes(i).pos(1:2) - sample(1:2));
            if d < min_dist
                min_dist = d;
                nearest_id = int32(i);
            end
        end
        nearest_node = nodes(nearest_id);

        theta = atan2(sample(2) - nearest_node.pos(2), sample(1) - nearest_node.pos(1));
        new_pos = [nearest_node.pos(1:2) + step_size * [cos(theta), sin(theta)], theta];

        if ~collision_check_line(nearest_node.pos, new_pos, obstacle_list, space)
            new_node = struct('pos', new_pos, ...
                'parent', int32(nearest_id), ...
                'cost', nearest_node.cost + norm(new_pos(1:2) - nearest_node.pos(1:2)));

            for i = 1:node_count
                if norm(nodes(i).pos(1:2) - new_node.pos(1:2)) < radius
                    temp_cost = nodes(i).cost + norm(nodes(i).pos(1:2) - new_node.pos(1:2));
                    if temp_cost < new_node.cost && ~collision_check_line(nodes(i).pos, new_node.pos, obstacle_list, space)
                        new_node.cost = temp_cost;
                        new_node.parent = int32(i);
                    end
                end
            end

            if node_count < MAX_NODES
                node_count = node_count + 1;
                nodes(node_count) = new_node;
            end

            if norm(new_node.pos(1:2) - goal(1:2)) < 25
                [valid, rs_path] = reeds_shepp_connect(new_node.pos, goal, R, obstacle_list, space);
                if valid
                    c_rs = new_node.cost + compute_rs_cost(rs_path);
                    if c_rs < c_best
                        c_best = c_rs;
                        best_goal_node = node_count;
                        rs_count = int32(min(size(rs_path,1), size(rs_to_goal,1)));
                        rs_to_goal(1:rs_count,:) = rs_path(1:rs_count,:);
                    end
                end
            end
        end
    end

    if best_goal_node > 0
        path_temp = zeros(300, 3);
        count = int32(0);
        current = best_goal_node;
        while current > 0
            count = count + 1;
            path_temp(count,:) = nodes(current).pos;
            current = nodes(current).parent;
        end
        rrt_path = flipud(path_temp(1:count,:));
        rrt_path = path_clothoid_smoothing(rrt_path); % 고급 smoothing 적용
        n_rrt = size(rrt_path,1);
        full_path = NaN(300,3);
        full_path(1:n_rrt,:) = rrt_path;
        full_path(n_rrt+1:n_rrt+rs_count,:) = rs_to_goal(1:rs_count,:);
    else
        full_path = NaN(300,3);
        disp('No Map Path!')
    end
    function r = rand_range(a, b) %#codegen
        r = a + (b - a) * rand;
    end
    disp("MAP function finish")
    %disp(g_x)
    %disp(g_y)
end
function sample = informed_sample(start, goal, c_max, space) %#codegen
    % === 고정된 출력: 반드시 [1×3] 벡터 ===
    sample = zeros(1, 3);

    % === 타원 파라미터 계산 ===
    c_min = norm(goal(1:2) - start(1:2));
    a = c_max / 2;
    b = sqrt(max(1e-6, c_max^2 - c_min^2)) / 2;
    theta = atan2(goal(2) - start(2), goal(1) - start(1));
    cos_t = cos(theta);
    sin_t = sin(theta);
    mid = 0.5 * (start(1:2) + goal(1:2));

    % === 고정 반복 횟수 시도 (예: 50회) ===
    for i = 1:50
        x = a * (2 * rand - 1);  % [-a, a]
        y = b * (2 * rand - 1);  % [-b, b]

        % 타원 내부 점만 사용
        if (x^2 / a^2 + y^2 / b^2) <= 1
            pos_x = cos_t * x - sin_t * y + mid(1);
            pos_y = sin_t * x + cos_t * y + mid(2);
            yaw   = 2 * pi * rand;

            sample(1) = pos_x;
            sample(2) = pos_y;
            sample(3) = yaw;
            return;
        end
    end

    % 실패 시 fallback: goal 반환
    sample = goal(:)';  % [1×3] 형태 유지
end

function smooth_path = path_clothoid_smoothing(path)
    if size(path, 1) < 3
        smooth_path = path;
        return;
    end

    fixed_points = false(size(path, 1), 1);
    for i = 2:size(path, 1) - 1
        prev = path(i - 1, 1:2);
        curr = path(i, 1:2);
        next = path(i + 1, 1:2);
        angle1 = atan2(curr(2) - prev(2), curr(1) - prev(1));
        angle2 = atan2(next(2) - curr(2), next(1) - curr(1));
        if abs(angle1 - angle2) < deg2rad(5)
            fixed_points(i) = true;
        end
    end

    x = path(:, 1); y = path(:, 2); yaw = path(:, 3);
    s = cumsum([0; sqrt(diff(x).^2 + diff(y).^2)]);
    ts = linspace(0, s(end), 5 * numel(s));
    x_smooth = interp1(s(~fixed_points), x(~fixed_points), ts, 'pchip', 'extrap');
    y_smooth = interp1(s(~fixed_points), y(~fixed_points), ts, 'pchip', 'extrap');
    yaw_unwrap = unwrap(yaw);
    yaw_smooth = interp1(s(~fixed_points), yaw_unwrap(~fixed_points), ts, 'pchip', 'extrap');

    dx = gradient(x_smooth); dy = gradient(y_smooth);
    ddx = gradient(dx); ddy = gradient(dy);
    curvature = abs((dx .* ddy - dy .* ddx) ./ (dx.^2 + dy.^2).^(3/2));
    max_curvature = 0.05;

    for k = 2:length(curvature)
        if curvature(k) > max_curvature
            factor = max_curvature / curvature(k);
            x_smooth(k) = x_smooth(k-1) + factor * (x_smooth(k) - x_smooth(k-1));
            y_smooth(k) = y_smooth(k-1) + factor * (y_smooth(k) - y_smooth(k-1));
        end
    end
    smooth_path = [x_smooth',...
                    y_smooth',...
                    wrapToPi(yaw_smooth')];
end

function [valid, rs_path] = reeds_shepp_connect(start, goal, R, obstacle_list, space)
    param.step = 0.5;
    param.max_curv = 1 / R;
    [x_list, y_list, yaw_list] = generation(start, goal, param);
    rs_path = [x_list(:), y_list(:), yaw_list(:)];
    valid = true;
    for i = 1:size(rs_path,1)
        pos = rs_path(i, :);
        if collision_check_point(pos, obstacle_list, space)
            valid = false;
            return;
        end
    end
end

function cost = compute_rs_cost(rs_path)
    % rs_path: [N x 3] 형태의 Reeds-Shepp 경로 (x, y, yaw)
    % 두 점 사이의 유클리디안 거리 누적합을 사용한 비용 계산
    if size(rs_path,1) < 2
        cost = 0;
        return;
    end
    dx = diff(rs_path(:,1));
    dy = diff(rs_path(:,2));
    dist = hypot(dx, dy);
    cost = sum(dist);
end

function is_collision = collision_check_line(start_pos, end_pos, obstacle_list, space) %#codegen
    % 간단한 선분-장애물 교차 검사: 샘플링 방식 사용
    n = 10;
    is_collision = false;
    for i = 0:n
        t = i / n;
        pos = (1 - t) * start_pos + t * end_pos;
        if collision_check_point(pos, obstacle_list, space)
            is_collision = true;
            return;
        end
    end
end

function is_collision = collision_check_point(pos, obstacle_list, space)
    x = pos(1); y = pos(2);
    if x < space(1) || x > space(2) || y < space(3) || y > space(4)
        is_collision = true;
        return;
    end
    is_collision = false;
    for i = 1:size(obstacle_list, 1)
        dx = x - obstacle_list(i,1);
        dy = y - obstacle_list(i,2);
        if hypot(dx, dy) < obstacle_list(i,3) + 1.5
            is_collision = true;
            return;
        end
    end
end

function [x_list, y_list, yaw_list] = generation(start_pose, goal_pose, param)
    sx = start_pose(1); sy = start_pose(2); syaw = start_pose(3);
    gx = goal_pose(1); gy = goal_pose(2); gyaw = goal_pose(3);

    dx = gx - sx; dy = gy - sy; dyaw = gyaw - syaw;
    x = (cos(syaw) * dx + sin(syaw) * dy) * param.max_curv;
    y = (-sin(syaw) * dx + cos(syaw) * dy) * param.max_curv;
    planner_fns = {@SCS, @CCC, @CSC, @CCCC, @CCSC};

    best_cost = inf;
    best_path = struct('segs', zeros(1,3), 'ctypes', ['L','S','L'], 'len', inf);

    % === 3) 각 플래너로부터 후보 경로 받아 최저 비용 갱신 ===
    for pi = 1:numel(planner_fns)
        paths = planner_fns{pi}(x, y, dyaw);
        for k = 1:numel(paths)
            if paths(k).len < best_cost
                best_cost = paths(k).len;
                best_path = paths(k);
            end
        end
    end

    % === 4) 유효한 경로가 하나도 없으면 빈 배열 리턴(선택사항) ===
    if isinf(best_cost)
        x_list = zeros(0,1); 
        y_list = zeros(0,1); 
        yaw_list = zeros(0,1);
        return;
    end

    points_num = floor(best_cost / param.step) + length(best_path.segs) + 5;
    x_list_ = zeros(points_num, 1);
    y_list_ = zeros(points_num, 1);
    yaw_list_ = zeros(points_num, 1);

    i = 1;
    x = 0; y = 0; yaw = 0;
    for j = 1:length(best_path.segs)
        m = best_path.ctypes(j);
        seg_length = best_path.segs(j);
        if seg_length == 0
            continue;   % 0 길이 세그먼트는 건너뛸 것
        end
        d_length = param.step * sign(seg_length);
        l = 0;
        while abs(l) <= abs(seg_length)
            new_pt = interpolate(m, l, [x, y, yaw], param);
            i = i + 1;
            x_list_(i) = new_pt(1);
            y_list_(i) = new_pt(2);
            yaw_list_(i) = new_pt(3);
            l = l + d_length;
        end
        new_pt = interpolate(m, seg_length, [x, y, yaw], param);
        i = i + 1;
        x_list_(i) = new_pt(1);
        y_list_(i) = new_pt(2);
        yaw_list_(i) = new_pt(3);
        x = new_pt(1); y = new_pt(2); yaw = new_pt(3);
    end

    x_list_ = x_list_(1:i);
    y_list_ = y_list_(1:i);
    yaw_list_ = yaw_list_(1:i);

    x_list = zeros(size(x_list_));
    y_list = zeros(size(y_list_));
    yaw_list = zeros(size(yaw_list_));
    for k = 1:length(x_list_)
        x_list(k) = cos(-syaw) * x_list_(k) + sin(-syaw) * y_list_(k) + sx;
        y_list(k) = -sin(-syaw) * x_list_(k) + cos(-syaw) * y_list_(k) + sy;
        yaw_list(k) = M(yaw_list_(k) + syaw);
    end
end

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
        tau = M(t1 + pi);
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

function path = RSPath(segs_in, ctypes_in)
    MAX_LEN = 5;  % 최대 5개 segment까지 허용
    segs_fixed = zeros(1, MAX_LEN);
    ctypes_fixed = repmat(' ', 1, MAX_LEN);

    n = length(segs_in);
    segs_fixed(1:n) = segs_in;

    for i = 1:n
        ctypes_fixed(i) = ctypes_in{i};
    end

    path.segs = segs_fixed;
    path.ctypes = ctypes_fixed;
    path.len = sum(abs(segs_in(:)));  % ✅ 항상 스칼라
end

    
function paths = SCS(x, y, phi) %#codegen
    MAX_PATHS = 2;
    paths = init_rs_paths(MAX_PATHS);
    cnt = uint8(0);

    segs = SLS(x, y, phi);
    if ~isempty(segs)
        cnt = cnt + 1;
        paths(cnt) = RSPath(segs, {'S', 'L', 'S'});
    end

    segs = SLS(x, -y, -phi);
    if ~isempty(segs)
        cnt = cnt + 1;
        paths(cnt) = RSPath(segs, {'S', 'R', 'S'});
    end

    paths = paths(1:cnt);
end

function paths = CCC(x, y, phi) %#codegen
    MAX_PATHS = 8;
    paths = init_rs_paths(MAX_PATHS);
    cnt = uint8(0);

    cases = {
        LRL(x, y, phi),            {'L','R','L'};
        -LRL(-x, y, -phi),         {'L','R','L'};
        LRL(x, -y, -phi),          {'R','L','R'};
        -LRL(-x, -y, phi),         {'R','L','R'};
    };

    for k = 1:size(cases,1)
        segs = cases{k,1};
        if ~isempty(segs)
            cnt = cnt + 1;
            paths(cnt) = RSPath(segs, cases{k,2});
        end
    end

    xb = x * cos(phi) + y * sin(phi);
    yb = x * sin(phi) - y * cos(phi);

    segs = LRL(xb, yb, phi);
    if ~isempty(segs)
        cnt = cnt + 1;
        paths(cnt) = RSPath([segs(3), segs(2), segs(1)], {'L','R','L'});
    end

    segs = LRL(-xb, yb, -phi);
    if ~isempty(segs)
        cnt = cnt + 1;
        paths(cnt) = RSPath(-[segs(3), segs(2), segs(1)], {'L','R','L'});
    end

    segs = LRL(xb, -yb, -phi);
    if ~isempty(segs)
        cnt = cnt + 1;
        paths(cnt) = RSPath([segs(3), segs(2), segs(1)], {'R','L','R'});
    end

    segs = LRL(-xb, -yb, phi);
    if ~isempty(segs)
        cnt = cnt + 1;
        paths(cnt) = RSPath(-[segs(3), segs(2), segs(1)], {'R','L','R'});
    end

    paths = paths(1:cnt);
end

function paths = CSC(x, y, phi) %#codegen
    MAX_PATHS = 8;
    paths = init_rs_paths(MAX_PATHS);
    cnt = uint8(0);

    segs_and_types = {
        LSL(x, y, phi),            {'L', 'S', 'L'};
        -LSL(-x, y, -phi),         {'L', 'S', 'L'};
        LSL(x, -y, -phi),          {'R', 'S', 'R'};
        -LSL(-x, -y, phi),         {'R', 'S', 'R'};
        LSR(x, y, phi),            {'L', 'S', 'R'};
        -LSR(-x, y, -phi),         {'L', 'S', 'R'};
        LSR(x, -y, -phi),          {'R', 'S', 'L'};
        -LSR(-x, -y, phi),         {'R', 'S', 'L'};
    };

    for k = 1:size(segs_and_types, 1)
        segs = segs_and_types{k,1};
        if ~isempty(segs)
            cnt = cnt + 1;
            paths(cnt) = RSPath(segs, segs_and_types{k,2});
        end
    end

    paths = paths(1:cnt);
end

function paths = CCCC(x, y, phi) %#codegen
    MAX_PATHS = 8;
    paths = init_rs_paths(MAX_PATHS);
    cnt = uint8(0);

    segs = LRLRn(x, y, phi);
    if ~isempty(segs)
        cnt = cnt + 1;
        paths(cnt) = RSPath([segs(1), segs(2), -segs(2), segs(3)], {'L','R','L','R'});
    end

    segs = LRLRn(-x, y, -phi);
    if ~isempty(segs)
        cnt = cnt + 1;
        paths(cnt) = RSPath([-segs(1), -segs(2), segs(2), -segs(3)], {'L','R','L','R'});
    end

    segs = LRLRn(x, -y, -phi);
    if ~isempty(segs)
        cnt = cnt + 1;
        paths(cnt) = RSPath([segs(1), segs(2), -segs(2), segs(3)], {'R','L','R','L'});
    end

    segs = LRLRn(-x, -y, phi);
    if ~isempty(segs)
        cnt = cnt + 1;
        paths(cnt) = RSPath([-segs(1), -segs(2), segs(2), -segs(3)], {'R','L','R','L'});
    end

    segs = LRLRp(x, y, phi);
    if ~isempty(segs)
        cnt = cnt + 1;
        paths(cnt) = RSPath([segs(1), segs(2), segs(2), segs(3)], {'L','R','L','R'});
    end

    segs = LRLRp(-x, y, -phi);
    if ~isempty(segs)
        cnt = cnt + 1;
        paths(cnt) = RSPath([-segs(1), -segs(2), -segs(2), -segs(3)], {'L','R','L','R'});
    end

    segs = LRLRp(x, -y, -phi);
    if ~isempty(segs)
        cnt = cnt + 1;
        paths(cnt) = RSPath([segs(1), segs(2), segs(2), segs(3)], {'R','L','R','L'});
    end

    segs = LRLRp(-x, -y, phi);
    if ~isempty(segs)
        cnt = cnt + 1;
        paths(cnt) = RSPath([-segs(1), -segs(2), -segs(2), -segs(3)], {'R','L','R','L'});
    end

    paths = paths(1:cnt);
end

function paths = CCSC(x, y, phi) %#codegen
    MAX_PATHS = 16;
    paths = init_rs_paths(MAX_PATHS);
    cnt = uint8(0);

    variants = {
        LRSL(x, y, phi),           {'L', 'R', 'S', 'L'}, [1 -0.5*pi 2 3];
        LRSL(-x, y, -phi),         {'L', 'R', 'S', 'L'}, [-1 0.5*pi -2 -3];
        LRSL(x, -y, -phi),         {'R', 'L', 'S', 'R'}, [1 -0.5*pi 2 3];
        LRSL(-x, -y, phi),         {'R', 'L', 'S', 'R'}, [-1 0.5*pi -2 -3];
        LRSR(x, y, phi),           {'L', 'R', 'S', 'R'}, [1 -0.5*pi 2 3];
        LRSR(-x, y, -phi),         {'L', 'R', 'S', 'R'}, [-1 0.5*pi -2 -3];
        LRSR(x, -y, -phi),         {'R', 'L', 'S', 'L'}, [1 -0.5*pi 2 3];
        LRSR(-x, -y, phi),         {'R', 'L', 'S', 'L'}, [-1 0.5*pi -2 -3];
    };

    for k = 1:size(variants,1)
        segs = variants{k,1};
        type = variants{k,2};
        idxs = variants{k,3};
        if ~isempty(segs)
            t = segs(abs(idxs(1))) * sign(idxs(1));
            u = segs(abs(idxs(3))) * sign(idxs(3));
            v = segs(abs(idxs(4))) * sign(idxs(4));
            paths(cnt + 1) = RSPath([t, idxs(2), u, v], type);
            cnt = cnt + 1;
        end
    end

    paths = paths(1:cnt);
end

function out = init_rs_paths(N)
    tmpl = struct('segs', zeros(1,5), 'ctypes', repmat(' ',1,5), 'len', 0);
    out = repmat(tmpl, N, 1);
end

function new_pt = interpolate(mode, length, init_pose, param)
    x = init_pose(1); y = init_pose(2); yaw = init_pose(3);

    if mode == 'S'
        new_x   = x + length / param.max_curv * cos(yaw);
        new_y   = y + length / param.max_curv * sin(yaw);
        new_yaw = yaw;
    elseif mode == 'L'
        new_x   = x + (sin(yaw + length) - sin(yaw)) / param.max_curv;
        new_y   = y - (cos(yaw + length) - cos(yaw)) / param.max_curv;
        new_yaw = yaw + length;
    elseif mode == 'R'
        new_x   = x - (sin(yaw - length) - sin(yaw)) / param.max_curv;
        new_y   = y + (cos(yaw - length) - cos(yaw)) / param.max_curv;
        new_yaw = yaw - length;
    else
        new_x = 0; new_y = 0; new_yaw = 0;
    end
    new_pt = [new_x, new_y, new_yaw];
end
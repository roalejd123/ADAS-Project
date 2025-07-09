function visualize_static_map()
    % ============================================
    % 🚗 맵 시각화: 장애물(머리좌표), 경계선, 시작/도착점
    % ============================================

    % === 1. 입력값 설정 ===
    map_boundary = [0, 0, 100, 0, 100, -100, 0, -100];  % 시계방향 사각형
    traffic_info = [40   -12   pi/2;
                    40   -24   pi/2;
                    40   -36   pi/2;
                    40   -48   pi/2;
                    39   -50   0;
                    51   -50   0;
                    63   -50   0];  % [x_head, y_head, yaw]
    traffic_size = [2.48, 11.5];    % [width, length] (m)
    start_point  = [0, -20];
    finish_point = [80, -3];

    % === 2. 시각화 시작 ===
    figure;
    hold on; axis equal; grid on;
    title('🚗 Map Visualization (Head-based obstacle coords)');
    xlabel('X [m]'); ylabel('Y [m]');

    % === 3. 경계선 시각화 ===
    xb = map_boundary(1:2:end);
    yb = map_boundary(2:2:end);
    xb(end+1) = xb(1);  % 선 닫기
    yb(end+1) = yb(1);
    plot(xb, yb, 'b-', 'LineWidth', 2);

    % === 4. 장애물 시각화 ===
    w = traffic_size(1);  % 차량 너비
    l = traffic_size(2);  % 차량 길이
    num_traffic = size(traffic_info,1);

    for i = 1:num_traffic
        % 🚛 차량 머리 좌표 및 방향
        x_head = traffic_info(i,1);
        y_head = traffic_info(i,2);
        yaw    = traffic_info(i,3);

        % ✅ 머리 좌표 → 중심 좌표 보정
        offset = [-cos(yaw); -sin(yaw)] ;  % 뒤로 이동
        xc = x_head - offset(1);
        yc = y_head - offset(2);

        % ✅ 로컬 좌표계의 꼭짓점 (중심 기준)
        dx = w; dy = l;
        local = [
            0, -dx;
            0, dx;
            dy, dx;
            dy, -dx]; % 다시 앞오른쪽 (닫기용)

        % ✅ 회전 적용
        R = [cos(yaw), -sin(yaw);
             sin(yaw),  cos(yaw)];
        global_P = (R * local')' + [xc, yc];

        % ✅ 시각화
        fill(global_P(:,1), global_P(:,2), 'r', ...
            'FaceAlpha', 0.4, 'EdgeColor', 'k');
    end

    % === 5. 시작점 / 목표점 표시 ===
    plot(start_point(1), start_point(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    text(start_point(1)+1, start_point(2), 'START', ...
         'Color', 'g', 'FontSize', 10, 'FontWeight', 'bold');

    plot(finish_point(1), finish_point(2), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
    text(finish_point(1)+1, finish_point(2), 'GOAL', ...
         'Color', 'b', 'FontSize', 10, 'FontWeight', 'bold');
end
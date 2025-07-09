function local_waypoints_ = global_to_local_waypoints(waypoints, vehicle_position, head)
    % 글로벌 좌표를 로컬 좌표로 변환해주는 함수
    % waypoints : Global_Waypoints [id, X, Y]
    % vehicle_position : [Ego_Global_X, Ego_Global_Y]
    % head : Ego_Yaw (rad)
    % 출력: Local Waypoints

    % 초기화
    num_points = size(waypoints, 1);
    local_waypoints_ = NaN(num_points, 2);  % [x_local, y_local]

    % 차량 위치 추출
    ego_x = vehicle_position(1);
    ego_y = vehicle_position(2);
    ego_yaw = head;

    % 회전 행렬 생성 (글로벌→로컬 변환)
    R = [cos(-ego_yaw) -sin(-ego_yaw); 
         sin(-ego_yaw)  cos(-ego_yaw)];

    count = 0;
    % 각 웨이포인트에 대해 변환 수행
    for i = 1:num_points
        % 글로벌 좌표에서 차량까지의 상대 위치
        dx = waypoints(i, 1) - ego_x;  % X 좌표 차이
        dy = waypoints(i, 2) - ego_y;  % Y 좌표 차이

        % 회전 변환 적용
        local_pos = R * [dx; dy];
        distance = sqrt(dx.^2 + dy.^2); % 차량으로부터 waypoint까지의 길이 계산

        % 시작 좌표 저장
        if count == 0 && local_pos(1) > 0 && distance < 15
            count = count + 1;
            local_waypoints_(count, :) = [local_pos(1) local_pos(2)];     % 로컬 x 좌표 
        end

        % 아래의 조건을 모두 만족하는 경우에만 좌표 저장
        % 1. x 좌표가 0보다 큰 경우
        % 2. x의 현재 좌표가 이전 x의 좌표보다 큰 경우
        % 3. 차량으로부터 waypoint까지의 길이가 30m 이내인 경우 (길이는 임의로 테스트 한 후 설정 가능할 듯)
        if count  ~= 0 && local_pos(1) > 0 && local_pos(1) > local_waypoints_(count, 1)  && distance < 15
            count = count + 1;
            local_waypoints_(count, :) = [local_pos(1) local_pos(2)];
        end
    end
end


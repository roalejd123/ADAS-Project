%% parking_Pathplanning.m
% 맵 생성 + 플롯 스크립트 (경계 샘플링 수정 반영)

% 1) 파라미터 정의
map_boundary = [0, 0, 100, 0, 100, -100, 0, -100];  % 시계방향 사각형 (8요소)
traffic_info = [ ...
    40, -12, pi/2;
    40, -24, pi/2;
    40, -36, pi/2;
    40, -48, pi/2;
    39, -50,    0;
    51, -50,    0;
    63, -50,    0 ];
traffic_size  = [2.48, 11.5];    % [width, length] (m)

% 2) 맵 생성
mapMatrix = generate_map_(map_boundary, traffic_info, traffic_size);

% 3) 플롯
plot_mapMatrix_true_axes(mapMatrix);


%% generate_map_ 함수
function mapMatrix = generate_map_(map_boundary, traffic_info, traffic_size)
%#codegen
% 맵 경계선 + 장애물 테두리 마스킹 (Y축 반전)
% map_boundary = [x1,y1, x2,y2, x3,y3, x4,y4]

    % === 초기화 ===
    mapMatrix = zeros(100,100);

    % === 장애물 테두리 처리 ===
    traffic_info = reshape(traffic_info,[],3);   % [N×3]
    num_traffic  = size(traffic_info,1);
    width  = traffic_size(1);
    length = traffic_size(2);
    dx     = width  / 2;
    dy     = length;        % head 기준 full length

    for i = 1:num_traffic
        xc  = traffic_info(i,1);
        yc  = traffic_info(i,2);
        yaw = traffic_info(i,3);

        % local 사각형 (head 기준)
        local = [ 0,   -dx;
                  0,    dx;
                  dy,   dx;
                  dy,  -dx ];

        R       = [cos(yaw), -sin(yaw);
                   sin(yaw),  cos(yaw)];
        pts     = (R * local')' + [xc, yc];   % 4×2
        global_pts = [pts; pts(1,:)];         % 5×2 닫힌 폴리곤

        % 테두리 따라 1 찍기
        Nline = 100;
        for k = 1:4
            x1 = global_pts(k  ,1);  y1 = global_pts(k  ,2);
            x2 = global_pts(k+1,1);  y2 = global_pts(k+1,2);
            xs = linspace(x1, x2, Nline);
            ys = linspace(y1, y2, Nline);
            for j = 1:Nline
                xi = round(xs(j));
                yi = round(-ys(j));      % Y 반전
                xi = min(max(xi,1),100);
                yi = min(max(yi,1),100);
                mapMatrix(yi, xi) = 1;
            end
        end
    end

    % === 맵 바운더리 테두리 처리 ===
    xb = map_boundary(1:2:end);  % [x1, x2, x3, x4]
    yb = map_boundary(2:2:end);  % [y1, y2, y3, y4]
    xb(end+1) = xb(1);           % 닫기
    yb(end+1) = yb(1);

    for k = 1:4
        x1  = xb(k);   y1 = yb(k);
        x2  = xb(k+1); y2 = yb(k+1);
        dx_ = abs(x2 - x1);
        dy_ = abs(y2 - y1);
        Nseg = max(dx_, dy_) + 1;  % 정확히 한 픽셀씩 간격

        xs = linspace(x1, x2, Nseg);
        ys = linspace(y1, y2, Nseg);
        for j = 1:Nseg
            xi = round(xs(j));
            yi = round(-ys(j));      % Y 반전
            xi = min(max(xi,1),100);
            yi = min(max(yi,1),100);
            mapMatrix(yi, xi) = 1;
        end
    end
end


%% 플롯 함수
function plot_mapMatrix_true_axes(mapMatrix)
    figure;
    imagesc([0,100], [-100,0], mapMatrix);  % X축 0~100, Y축 -100~0
    colormap(gray);
    axis equal tight;
    title('Map Matrix (True Coordinates)');
    xlabel('X [m]');
    ylabel('Y [m]');
end

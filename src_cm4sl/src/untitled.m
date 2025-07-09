%==========================================================================
% orderedPath1 타임시리즈에서 연속 포인트 간 거리 계산 및 안전 출력
%==========================================================================

% 1) timeseries에서 데이터 추출
ts   = orderedPath1;      % orderedPath1 timeseries 객체
data = ts.Data;           % N×3 배열 ([x, y, yaw])
time = ts.Time;           % N×1 벡터 (시간)

% 2) x, y 좌표 분리
x = data(:,1);
y = data(:,2);

% 3) 포인트 수 체크
N = numel(x);
if N < 2
    fprintf('orderedPath1에 포인트가 %d개뿐이어서 거리 계산을 할 수 없습니다.\n', N);
    return;
end

% 4) 연속된 점 사이 거리 계산
dists = zeros(N-1,1);
for i = 1:N-1
    dx       = x(i+1) - x(i);
    dy       = y(i+1) - y(i);
    dists(i) = sqrt(dx^2 + dy^2);
end

% 5) 결과 안전 출력
fprintf('=== 연속 포인트 간 거리 (총 %d개 계산) ===\n', N-1);
for i = 1:N-1
    % time(i)와 time(i+1)가 모두 유효한지 확인
    if numel(time) >= i+1
        fprintf('포인트 %2d → %2d: 거리 = %.3f m (t=%.2f→%.2f s)\n', ...
                i, i+1, dists(i), time(i), time(i+1));
    else
        % 시간이 모자라면 거리만 출력
        fprintf('포인트 %2d → %2d: 거리 = %.3f m\n', ...
                i, i+1, dists(i));
    end
end

% 6) (선택) 거리 시각화
figure;
plot(1:N-1, dists, '-o', 'LineWidth',1.5);
xlabel('구간 인덱스 i → i+1');
ylabel('거리 [m]');
title('orderedPath1 연속 포인트 간 거리');
grid on;

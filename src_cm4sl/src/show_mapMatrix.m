%% plot_out_mapMatrix.m
% out_mapMatrix라는 timetable의 각 프레임을 순차적으로 플롯해서 애니메이션처럼 보여줍니다.

% 1) 데이터형에 따라 한 프레임을 꺼내오는 함수 정의
if iscell(out_mapMatrix.Data)
    % Data가 cell 배열에 각 원소가 100×100 double일 때
    getFrame = @(i) out_mapMatrix.Data{i};
else
    % Data가 3차원 numeric 배열 (N×100×100) 일 때
    tmp = out_mapMatrix.Data;  % N×100×100
    getFrame = @(i) squeeze(tmp(i,:,:));
end

% 2) 시간축과 전체 스텝 수
tt = out_mapMatrix.Time;         % 시뮬레이션 시간(Duration 배열)
N  = height(out_mapMatrix);      % 프레임 개수

% 3) 플롯 초기화
figure('Name','MapMatrix Animation','NumberTitle','off');
colormap(gray);

% 4) 순차 플롯 루프
for k = 1:N
    M = getFrame(k);  % 100×100 mapMatrix
    
    % 실제 좌표계: X 0~100, Y -100~0
    imagesc([0,100], [-100,0], M);
    axis equal tight;
    xlabel('X [m]');
    ylabel('Y [m]');
    
    % 제목에 시간 표시
    t_sec = seconds(tt(k));  % Duration → 초
    title(sprintf('Map at t = %.3f s', t_sec));
    
    drawnow;    % 화면 갱신
    % pause(0.01);  % 원하시면 프레임 간 딜레이 추가
end

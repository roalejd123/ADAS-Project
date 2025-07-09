visualize_path_and_obstacles(path1, obstacles1);

function visualize_path_and_obstacles(path1, obstacles1)
    % 시각화 시작
    figure;
    hold on;
    xlabel('X Position');
    ylabel('Y Position');
    title('Hybrid A* Path with obstacles');
    grid on;
    axis equal;

    %% === 경로 시각화 ===
    if isa(path1, 'timeseries')
        data = path1.Data;                   
        final_path = squeeze(data(:,:,end)); 
    elseif isnumeric(path1) && ndims(path1)==2 && size(path1,2)>=2
        final_path = path1;
    else
        error('path1은 timeseries 또는 [N×2]·[N×3] 배열이어야 합니다.');
    end

    final_path = final_path(~isnan(final_path(:,1)), :);

    %경로 그리기 코드
    start_pt = final_path(1,1:2);
    goal_pt  = final_path(end,1:2);

    plot(final_path(:,1), final_path(:,2), '-o','LineWidth',2,'MarkerSize',4,'Color','b');
    plot(start_pt(1), start_pt(2), 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
    plot(goal_pt(1),  goal_pt(2),  'ro', 'MarkerSize',10,'MarkerFaceColor','r');

    %% === 장애물 시각화 ===
    if isa(obstacles1, 'timeseries')
        data_obs = obstacles1.Data;                     
        final_obs = squeeze(data_obs(:,:,end));         
    elseif isnumeric(obstacles1) && ndims(obstacles1)==2 && size(obstacles1,2)>=3
        final_obs = obstacles1;
    else
        error('obstacles1은 timeseries 또는 [M×3] 배열이어야 합니다.');
    end

    obs = final_obs(final_obs(:,3)>0, :);
    theta = linspace(0,2*pi,50);
    for i=1:size(obs,1)
        x = obs(i,1);  y = obs(i,2);  r = obs(i,3);
        xc = x + r*cos(theta);
        yc = y + r*sin(theta);
        plot(xc, yc, 'r-', 'LineWidth',1.5);
        plot(x,  y,  'ro', 'MarkerSize',5,'MarkerFaceColor','r');
    end

    axis([2 48 -45 -4]);
    hold off;
end
function [local_point, dist] = global_to_local_waypoints(waypoints_x, waypoints_y, vehicle_position, head)
% 전역 좌표 → ego 로컬 좌표 변환
% Inputs:
%   waypoints_x, waypoints_y : 목표점 전역 X,Y 벡터
%   vehicle_position         : [ego_X, ego_Y]
%   head                     : ego_Yaw (rad)
% Outputs:
%   local_point              : [x_rel; y_rel] in ego frame
%   dist                     : Euclidean distance

    dx = waypoints_x - vehicle_position(1);
    dy = waypoints_y - vehicle_position(2);
    R = [ cos(head),  sin(head);
         -sin(head),  cos(head) ];
    rel = R * [dx; dy];
    local_point = rel;
    dist = sqrt(rel(1,:).^2 + rel(2,:).^2);
end

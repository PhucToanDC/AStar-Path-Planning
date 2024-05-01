clear all
close all

%Example 1
% map = false(10,10); 
% % mark obstacles
% map(1:7,3) = true;
% map(5,5:7) = true;
% map(3:10,7) = true;
% % assign start and goal node
% start_coords = [1, 1];
% goal_coords  = [7, 10];

%Example 2
map = false(17,17); 
% mark obstacles
map(1:7,3) = true;
map(5,5:7) = true;
map(3:10,7) = true;
map(11,1:12) = true;
map(7:10,12) = true;
map(15,8:14) = true;

stayAwayFromObstacles = true;
% assign start and goal node
start_coords = [1, 1];
goal_coords  = [17, 10];

%Example 3
% map = false(20,20); 
% % mark obstacles
% map(1,1:20) = true;
% map(1:20,1) = true;
% map(20,1:20) = true;
% map(1:20,20) = true;
% map(16,1:5) = true;
% map(12:15,5) = true;
% % assign start and goal node
% start_coords = [2, 2];
% goal_coords  = [19, 19];

% Boolian variable to control if the map is being visualized on every iteration
drawMapEveryTime = true;

[route] = myA_star(map,start_coords,goal_coords,drawMapEveryTime,stayAwayFromObstacles)


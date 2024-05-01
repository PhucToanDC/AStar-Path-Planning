function [route] = myA_star (input_map, start_coords, dest_coords, drawMapEveryTime, stayAwayFromObstacles)
% Run Dijkstra's algorithm on a grid.
% Basic Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%   the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%   respectively, the first entry is the row and the second the column.
% Basic Output :
%    route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route. This is a single dimensional vector
%    Step: Remember to also return the total number of nodes
%    expanded during your search. Do not count the goal node as an expanded node.

% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red - visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination

% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red - visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination

cmap = [1 1 1; ... % white
        0 0 0; ... % black
        1 0 0; ... % red
        0 0 1; ... % blue
        0 1 0; ... % green
        1 1 0; ... % yellow
	0.5 0.5 0.5];  % gray

colormap(cmap);

[nrows, ncols] = size(input_map);

map = zeros(nrows,ncols);

map(~input_map) = 1;   % Mark free cells with white
map(input_map)  = 2;   % Mark obstacle cells with black

% Generate linear indices of start and dest nodes
% sub2ind = (col_index - 1) * col size + row_index = (9-1)*10+8 = 88
start_node = sub2ind(size(input_map), start_coords(1), start_coords(2));
goal_node  = sub2ind(size(input_map), dest_coords(1),  dest_coords(2));

map(start_node) = 5; % mark start node with green
map(goal_node)  = 6; % mark dest node with yellow

G_cost = Inf(size(input_map));
H_cost = zeros(size(input_map));
F_cost = Inf(size(input_map));
F_cost_open_node = Inf(size(input_map));
G_cost_calculating = Inf(size(input_map));

for node = 1 : numel(input_map)
    [node_coords(1), node_coords(2)] = ind2sub(size(input_map), node);
    H_cost(node) = sqrt(abs(dest_coords(1)-node_coords(1))^2+abs(dest_coords(2)-node_coords(2))^2);
end

%start node
G_cost(start_node) = 0;
F_cost(start_node) = G_cost(start_node) + H_cost(start_node);

%parents
parent = zeros(size(input_map));

%Open list
Open_list = Inf(size(input_map));
%Close list
Close_list = Inf(size(input_map));
%Obstalces neighbor list
ObstacleNeighborList = zeros(size(input_map));
ObstacleNeighborCost = zeros(size(input_map));
if (stayAwayFromObstacles)
   for obstacle = 1:numel(input_map)
        if (input_map(obstacle))
            [obstacle_row, obstacle_col] = ind2sub(size(input_map),obstacle);
            for n = 1:4
                if n == 1
                    row = obstacle_row - 1; col = obstacle_col;
                elseif n == 2
                    row = obstacle_row + 1; col = obstacle_col;
                elseif n == 3
                    row = obstacle_row; col = obstacle_col - 1;
                elseif n == 4
                    row = obstacle_row; col = obstacle_col + 1;
                elseif n == 5
                    row = obstacle_row - 1; col = obstacle_col - 1;
                elseif n == 6
                    row = obstacle_row - 1; col = obstacle_col + 1;
                elseif n == 7
                    row = obstacle_row + 1; col = obstacle_col - 1;
                else
                    row = obstacle_row + 1; col = obstacle_col + 1;
                end
                %if out of the map -> Skip it
                if (row < 1 || row > size(input_map,1))
                    continue
                elseif (col < 1 || col > size(input_map,2))
                    continue
                end
                obstacle_neighbor = sub2ind(size(input_map),row,col);
                if (obstacle_neighbor == start_node)
                    continue
                end
                if (obstacle_neighbor == goal_node)
                    continue
                end
                ObstacleNeighborList(obstacle_neighbor) = 1;
                if (n <= 4)
                    ObstacleNeighborCost(obstacle_neighbor) = 1;
                elseif (n >= 5)
                    ObstacleNeighborCost(obstacle_neighbor) = 1;
                end
            end
        end
    end
end

%add start node to open
Open_list(start_node) = 1;
F_cost_open_node(start_node)=F_cost(start_node)*Open_list(start_node);

%keep track of number of nodes expanded
Step = 0;

%Main loop
while true

    map(start_node) = 5;
    map(goal_node) = 6;

    if(drawMapEveryTime)
         pause(0.1);
        image(1.50, 1.50, map);
        grid on; % show grid
        axis image; 
        drawnow;
    end
    

    %check node in open list with the lowest Fcost
    for open_node = 1 : numel(Open_list)
        F_cost_open_node(open_node) = F_cost(open_node)*Open_list(open_node);
    end
    [min_F, current_node] = min(F_cost_open_node(:));

    if ((current_node == goal_node) || isinf(min_F))
        break;
    end
    %remove current node from Open
    Open_list(current_node) = Inf;

    %add current node to Close
    Close_list(current_node) = 1;

    %Update map
    map(current_node) = 3; %red color

    [i,j] = ind2sub(size(input_map), current_node);

    %visit each neighbor of the current node
    for n = 1 : 8 %each cell has 8 neighbors, row and col are neighbor's coords
        if n == 1
            row = i - 1; col = j;
        elseif n == 2
            row = i + 1; col = j;
        elseif n == 3
            row = i; col = j - 1;
        elseif n == 4
            row = i; col = j + 1;
        elseif n == 5
            row = i - 1; col = j - 1;
        elseif n == 6
            row = i - 1; col = j + 1;
        elseif n == 7
            row = i + 1; col = j - 1;
        else
            row = i + 1; col = j + 1;
        end
%if out of the map -> Skip it
        if (row < 1 || row > size(input_map,1))
            continue
        elseif (col < 1 || col > size(input_map,2))
            continue
        end
        %set neighbor
        neighbor = sub2ind(size(input_map),row,col);
        %if the neighbor is obstacle or in Close -> skip
        if (input_map(neighbor) == 1)
            continue
        end

        if (Close_list(neighbor) == 1)
            continue
        end

        G_cost_calculating(neighbor) = G_cost(current_node) + sqrt(abs(i-row)^2+abs(j-col)^2) + ObstacleNeighborCost(neighbor);
        if (G_cost_calculating(neighbor)<G_cost(neighbor))
            G_cost(neighbor) = G_cost_calculating(neighbor);
            parent(neighbor) = current_node; %set current to parent of neighbor
        end

        F_cost(neighbor) = G_cost(neighbor) + H_cost(neighbor);
        
        if (Open_list(neighbor) == Inf)
            Open_list(neighbor) = 1; %add neighbor to open list
        end
        Step = Step + 1;
        % 2 update the map,
        if(map(neighbor) ~= 6) % goal should be always yellow
            map(neighbor) = 4; % mark neigbor with blue (to be visited) 
        end
        
        % nodes are expanded on the grid. 
        if (drawMapEveryTime)
            image(1.50, 1.50, map);
            grid on; % show grid
            axis image; 
            drawnow;
        end
    end 
end

%% Construct route from start to dest by following the parent links
if(isinf(F_cost(goal_node)))
    route = [];
    
else
    node = goal_node;
    route = [node];
    tot_cost = 0;
    while (true)
        node = parent(node);
        route = [route node];
        if node == start_node
            break
        end
        
    end
    for k = 2:length(route) - 1        
        map(route(k)) = 7;
        pause(0.1);
        image(1.5, 1.5, map);
        grid on;
        axis image;
    end
end

end

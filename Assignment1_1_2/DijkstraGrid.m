function [route,numExpanded] = DijkstraGrid (input_map, start_coords, dest_coords)
% Run Dijkstra's algorithm on a grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%   the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%   respectively, the first entry is the row and the second the column.
% Output :
%    route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route. This is a single dimensional vector
%    numExpanded: Remember to also return the total number of nodes
%    expanded during your search. Do not count the goal node as an expanded node.


% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination

cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0; ...
	0.5 0.5 0.5];

colormap(cmap);

% variable to control if the map is being visualized on every
% iteration
drawMapEveryTime = true;

[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;   % Mark free cells
map(input_map)  = 2;   % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

% Initialize distance array
distanceFromStart = Inf(nrows,ncols);

% For each grid cell this array holds the index of its parent
parent = zeros(nrows,ncols);

distanceFromStart(start_node) = 0;

% keep track of number of nodes expanded 
numExpanded = -1;

% Main Loop
while true
    
    % Draw current map
    map(start_node) = 5;
    map(dest_node) = 6;
    
    % make drawMapEveryTime = true if you want to see how the 
    % nodes are expanded on the grid. 
    if (drawMapEveryTime)
        image(1.5, 1.5, map);
        grid on;
        axis image;
        drawnow;
    end
    
    % Find the node with the minimum distance
    [min_dist, current] = min(distanceFromStart(:));

    if ((current == dest_node) || isinf(min_dist))
        break;
    end
    
    % Update map
    map(current) = 3;         % mark current node as visited
    distanceFromStart(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(distanceFromStart), current);
    
   % ********************************************************************* 
    % YOUR CODE BETWEEN THESE LINES OF STARS
        
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
    % set up color map for display

    % Upper neighbour 
    if (i-1 >= 1 && i-1 <= nrows  && map(i-1,j) ~= 2 && map(i-1,j) ~= 3 && map(i-1,j) ~= 5)
        map(i-1,j) = 4;
        distanceFromStart(i-1,j) = 1 + min_dist;
        parent(i-1,j) = current;
       % parent(i-1, j) = sub2ind(size(map), i, j);
    end
    % lower neighbor
    if (i + 1 >= 1 && i+1 <= nrows  && map(i+1,j) ~= 2 && map(i+1,j) ~= 3 && map(i+1,j) ~= 5)
        map(i+1,j) = 4;
        distanceFromStart(i+1,j) = 1 + min_dist;
        parent(i+1,j) = current;
       % parent(i+1, j) = sub2ind(size(map), i, j);
    end
    % left neighbor
    if (j-1 >= 1 && j-1 <= ncols  && map(i,j-1) ~= 2  && map(i,j-1) ~= 3 && map(i,j-1) ~= 5)
        map(i,j-1) = 4;
        distanceFromStart(i,j-1) = 1 + min_dist;
        parent(i,j-1) = current;
       % parent(i, j-1) = sub2ind(size(map), i, j);
    end
    % right neighbor
    if (j+1 >= 1 && j+1 <= ncols  && map(i,j+1) ~= 2  && map(i,j+1) ~= 3 && map(i,j+1) ~= 5 )
        map(i,j+1) = 4;
        distanceFromStart(i,j+1) = 1 + min_dist;
        parent(i,j+1) = current;
        %parent(i, j+1) = sub2ind(size(map), i, j);
    end
    
    numExpanded = numExpanded + 1;
    %*********************************************************************

end



%% Construct route from start to dest by following the parent links
if (isinf(distanceFromStart(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
    
        % Snippet of code used to visualize the map and the path
    for k = 2:length(route) - 1        
        map(route(k)) = 7;
        pause(0.1);
        image(1.5, 1.5, map);
        grid on;
        axis image;
    end
end

end

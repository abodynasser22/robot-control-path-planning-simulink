% astar_shortest_p1_clean_legend.m
% 1. Clean & Denoise Data
% 2. High-Contrast Visualization
% 3. Detailed Legends

close all; clear; clc;

%% --- PARAMETERS ---
res = 0.05;               
start_world = [0.0,  0.0];
goal_world  = [2.3, -1.4]; 
inflation_radius_m = 0.05;

%% --- 1. LOAD & CROP ---
if isfile('P1_cropped.mat')
    disp('Loading pre-cropped P1...');
    S = load('P1_cropped.mat');
    points = S.points;
else
    if ~isfile('P1.mat'), error('P1.mat not found.'); end
    S = load('P1.mat');
    vars = fieldnames(S);
    points_raw = double(S.(vars{1}));
    
    % Tight Crop
    crop_mask = points_raw(:,1) >= -0.2 & points_raw(:,1) <= 2.4 & ...
                points_raw(:,2) >= -2.2 & points_raw(:,2) <= 0.5;
    points = points_raw(crop_mask, :);
    save('P1_cropped.mat', 'points');
end

%% --- 2. CLEANING & OUTLIER REJECTION ---
% Radius Outlier Removal
r_noise = 0.1; min_neighbors = 70;
idx = rangesearch(points, points, r_noise);
keep_mask = cellfun(@numel, idx) > min_neighbors;
points_clean = points(keep_mask, :);

% Grid Downsampling (Thinning)
sub_res = 0.01; 
points_final = unique(round(points_clean./sub_res).*sub_res, 'rows');
fprintf('Points Cleaned: %d -> %d\n', length(points), length(points_final));

%% --- 3. GRID & MAP GENERATION ---
map_snap = round(points_final./res).*res;
xmin = min(map_snap(:,1)) - 0.5; xmax = max(map_snap(:,1)) + 0.5;
ymin = min(map_snap(:,2)) - 0.5; ymax = max(map_snap(:,2)) + 0.5;

nx = ceil((xmax-xmin)/res) + 1;
ny = ceil((ymax-ymin)/res) + 1;

cols = round((map_snap(:,1)-xmin)/res) + 1;
rows = round((map_snap(:,2)-ymin)/res) + 1;
cols = max(1, min(cols, nx)); rows = max(1, min(rows, ny));

occ_raw = false(ny,nx);
occ_raw(sub2ind([ny,nx], rows, cols)) = true;
occ_solid = imclose(occ_raw, strel('square', 2));

%% --- 4. MASKING & SAFETY ---
start_node = world2grid(start_world, xmin, ymin, res);
goal_node  = world2grid(goal_world, xmin, ymin, res);

% Flood Fill
reachable_mask = imfill(~occ_solid, [start_node(2), start_node(1)], 4); 
occ_hard_walls = occ_solid | ~reachable_mask;

% Inflation
inflate_cells = ceil(inflation_radius_m / res);
occ_inflated = imdilate(occ_hard_walls, strel('disk', inflate_cells));

occ_planning = occ_inflated;
occ_planning = force_clear(occ_planning, start_node, 2);
occ_planning = force_clear(occ_planning, goal_node, 2);

%% --- 5. A* PLANNER ---
disp('Planning A*...');
[path, g_score_map] = astar_search(occ_planning, start_node, goal_node);
if isempty(path), error('Path planning failed.'); end

%% --- 6. VISUALIZATION ---
figure(1); clf; set(gcf, 'Color', 'w');

% Prepare Heatmap Data
plot_data = g_score_map;
plot_data(isinf(plot_data)) = NaN; 
max_dist = max(plot_data(:));
val_safety = max_dist * 1.1; 
val_wall   = max_dist * 1.2; 

plot_data(occ_inflated) = val_safety; 
plot_data(occ_hard_walls) = val_wall;

imagesc([xmin xmax], [ymin ymax], plot_data);
set(gca,'YDir','normal'); axis equal; 

% Custom Colors
n_grad = 256;
grad_map = sky(n_grad); 
orange_color = [1 0.6 0]; % Safety
red_color    = [0.8 0 0]; % Wall

cmap = [grad_map; orange_color; red_color];
colormap(cmap); caxis([0, val_wall]); 

cb = colorbar; ylabel(cb, 'Distance Cost');
hold on;

% --- PLOT ELEMENTS ---
% Path
px = xmin + (path(:,1)-1)*res;
py = ymin + (path(:,2)-1)*res;
h_path = plot(px, py, 'g-', 'LineWidth', 4); 

% Start (Cyan Circle)
h_start = plot(start_world(1), start_world(2), 'co', ...
    'MarkerSize', 12, 'MarkerFaceColor', 'c', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

% Goal (Magenta Hexagram)
h_goal = plot(goal_world(1), goal_world(2), 'mh', ...
    'MarkerSize', 15, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

% Dummy Handles for Legend (since imagesc doesn't support legends directly)
h_wall = patch(NaN, NaN, red_color, 'EdgeColor', 'none'); 

% --- LEGEND ---
legend([h_wall, h_path, h_start, h_goal], ...
       {'Obstacles (Walls)', 'Optimal Path', 'Start', 'Goal'}, ...
       'Location', 'bestoutside', 'FontSize', 10);

title('Filtered Lidar Map & A* Path', 'FontSize', 14);
xlabel('X (m)'); ylabel('Y (m)');

%% --- 7. EXPORT WAYPOINTS ---
path_len = sum(sqrt(diff(px).^2 + diff(py).^2));
spacing_target = 0.3; 
k = max(2, round(length(px) / (path_len / spacing_target)));

mask = false(size(px));
mask(1) = true; mask(1:k:end) = true; mask(end) = true;
final_px = px(mask); final_py = py(mask);

% Small dots for waypoints
plot(final_px, final_py, 'w.', 'MarkerSize', 10, 'HandleVisibility', 'off');

waypoints_world = [final_px, final_py];
save('final_waypoints.mat', 'waypoints_world');
fprintf('Saved %d waypoints.\n', length(final_px));

%% --- FUNCTIONS ---
function [path, g] = astar_search(occ_map, start, goal)
    [rows, cols] = size(occ_map);
    [X, Y] = meshgrid(1:cols, 1:rows);
    h = sqrt((X-goal(1)).^2 + (Y-goal(2)).^2);
    g = inf(rows, cols); f = inf(rows, cols);
    g(start(2), start(1)) = 0; f(start(2), start(1)) = h(start(2), start(1));
    parent_x = zeros(rows, cols); parent_y = zeros(rows, cols);
    openList = [f(start(2),start(1)), start(1), start(2)];
    closed = false(rows, cols); found = false;
    while ~isempty(openList)
        [~, idx] = min(openList(:,1));
        cur = openList(idx,:); cx = cur(2); cy = cur(3);
        openList(idx,:) = [];
        if closed(cy,cx), continue; end
        closed(cy,cx) = true;
        if cx==goal(1) && cy==goal(2), found = true; break; end
        dirs = [-1 -1; 0 -1; 1 -1; -1 0; 1 0; -1 1; 0 1; 1 1];
        for i=1:8
            nx = cx + dirs(i,1); ny = cy + dirs(i,2);
            if nx<1 || nx>cols || ny<1 || ny>rows, continue; end
            if occ_map(ny,nx) || closed(ny,nx), continue; end
            tg = g(cy,cx) + sqrt(dirs(i,1)^2 + dirs(i,2)^2);
            if tg < g(ny,nx)
                g(ny,nx) = tg; f(ny,nx) = tg + h(ny,nx);
                parent_x(ny,nx) = cx; parent_y(ny,nx) = cy;
                openList = [openList; f(ny,nx), nx, ny]; 
            end
        end
    end
    if found
        path = [goal(1), goal(2)]; curr = goal;
        while ~isequal(curr, start)
            px = parent_x(curr(2), curr(1)); py = parent_y(curr(2), curr(1));
            curr = [px, py]; path = [curr; path];
        end
    else, path = []; end
end

function node = world2grid(p, xmin, ymin, res)
    col = round((p(1)-xmin)/res) + 1;
    row = round((p(2)-ymin)/res) + 1;
    node = [col, row];
end

function map = force_clear(map, center, r)
    [ny, nx] = size(map);
    xr = max(1, center(1)-r):min(nx, center(1)+r);
    yr = max(1, center(2)-r):min(ny, center(2)+r);
    map(yr, xr) = false;
end
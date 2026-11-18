

P = maze_points;

% Ta bort ensamma punkter (outliers) med radie-närmaste-grannar
r = 0.1;          % radie i meter (justera 0.1–0.25)
minNbr = 70;        % min antal grannar inom r

[idx, dist] = rangesearch(P, P, r);
keep = cellfun(@numel, idx) >= minNbr;

maze_points = P(keep,:);
figure; scatter(maze_points(:,1), maze_points(:,2), 3, 'filled');
title('Efter outlier-filter');
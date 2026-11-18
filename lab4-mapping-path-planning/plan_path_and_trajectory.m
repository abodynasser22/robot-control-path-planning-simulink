%% plan_path_and_trajectory.m
% Körs efter Task 2-simuleringen.
% Bygger occupancyMap (om behövs), planerar bana med PRM och skapar ref_traj
% för Simulink (From Workspace).

%% === Steg 2: Skapa / hämta occupancy-karta ==========================

% Ange samma upplösning som i Task 2 (celler per meter)
resolution = 10;        % EXEMPEL: 10 celler/m -> 0.1 m/cell

% Försök använda befintligt occupancyMap om det finns
if exist("map","var") && isa(map,"occupancyMap")
    omap = map;
    fprintf("Använder befintligt occupancyMap-objekt 'map'.\n");
    
elseif exist("mapMatrix","var")
    % mapMatrix antas vara en 2D-matris 0..1 från Task 2
    omap = occupancyMap(mapMatrix, resolution);
    fprintf("Skapar occupancyMap från 'mapMatrix' med resolution = %.2f celler/m.\n", resolution);
    
else
    error("Varken 'map' (occupancyMap) eller 'mapMatrix' finns i workspace. Kör Task 2 först.");
end

% Visa kartan för kontroll
figure; show(omap); title("Occupancy map - utläsning"); grid on;


%% === Steg 3: Planera bana med PRM (map-baserad planner) ==============

% Tjocka till väggarna med robotens radie (säkerhetsmarginal)
robotRadius = 0.15;      % [m] – anpassa till Turtlebot/robot
inflate(omap, robotRadius);

% Start- och målkoordinater i världskoordinater (meter)
start = [0 0];           % enligt uppgiften
goal  = [1.5 2.0];       % EXEMPEL – ändra vid behov

fprintf("Start = [%.2f %.2f], Goal = [%.2f %.2f]\n", start(1), start(2), goal(1), goal(2));

% Skapa PRM-planerare
numNodesBase = 500;
maxNodes     = 5000;
connectionDist = 0.5;    % [m] max kantlängd i grafen

path = [];
numNodes = numNodesBase;

while isempty(path) && numNodes <= maxNodes
    fprintf("Försöker bygga PRM med %d noder...\n", numNodes);
    
    prm = mobileRobotPRM(omap, numNodes);
    prm.ConnectionDistance = connectionDist;
    
    path = findpath(prm, start, goal);
    
    if isempty(path)
        fprintf("Ingen bana hittades med %d noder. Ökar antalet noder.\n", numNodes);
        numNodes = numNodes + 500;
    end
end

if isempty(path)
    error("Kunde inte hitta någon bana. Justera start/goal eller PRM-parametrar.");
else
    fprintf("Bana hittad med %d punkter.\n", size(path,1));
end

% Plotta banan i kartan
figure; show(omap); hold on;
plot(path(:,1), path(:,2), "LineWidth", 2);
plot(start(1), start(2), "go", "MarkerSize", 8, "LineWidth", 2);
plot(goal(1),  goal(2),  "rx", "MarkerSize", 8, "LineWidth", 2);
title("Planerad PRM-bana i kartan");
legend("Bana","Start","Goal");
hold off;


%% === Steg 4: Generera tidsparametriserad traj med mstraj =============

% Parametrar till mstraj
dt   = 0.1;             % tidssteg [s] (matcha Simulink-steget)
vmax = 0.2;             % max-hastighet [m/s]
tacc = 0.5;             % accelerationstid [s]

% Dela upp path i start och via-punkter
q0  = path(1,:);        % [1 x 2], startpunkt
via = path(2:end,:);    % [N-1 x 2], via-punkter

% mstraj för 2D (x,y)
% vmax*ones(1,2) => samma maxhastighet i x- och y-led
qtraj = mstraj(via, vmax*ones(1,2), [], q0, dt, tacc);

% Bygg tidvektor
K = size(qtraj,1);
t = (0:K-1)' * dt;      % [K x 1]

% Extrahera x, y
x = qtraj(:,1);
y = qtraj(:,2);

% Beräkna yaw från derivatan (riktningen längs trajen)
dx = [diff(x); 0];
dy = [diff(y); 0];
yaw = atan2(dy, dx);

% Bygg referenstabell till Simulink: [time, x_ref, y_ref, yaw_ref]
ref_traj = [t, x, y, yaw];

% Spara några variabler i base workspace (för From Workspace i Simulink)
assignin("base", "ref_traj", ref_traj);
assignin("base", "path_planned", path);

fprintf("ref_traj skapat (storlek %d x %d) och placerat i base workspace.\n", size(ref_traj,1), size(ref_traj,2));
fprintf("Använd 'ref_traj' i From Workspace-blocket i Simulink.\n");

%% (Valfritt) snabb plot av trajen
figure;
plot(x, y, "-o"); axis equal;
xlabel("x [m]"); ylabel("y [m]");
title("mstraj-trajectorie (x,y)");
grid on;

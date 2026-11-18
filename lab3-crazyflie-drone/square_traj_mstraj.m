% build_square_refs.m
L     = 0.7;
z0    = 1.0;
v     = 0.2;
Thold = 2.0;
dt    = 0.01;
tacc  = 0.2;

P1 = [0 0];
P2 = [L 0];
P3 = [L L];
P4 = [0 L];

P    = [P2; P2; P3; P3; P4; P4; P1; P1];
Tedge = L / v;
Tseg  = [Tedge Thold Tedge Thold Tedge Thold Tedge Thold];

posRef = mstraj(P, [], Tseg, P1, dt, tacc);  % Nx2
N      = size(posRef,1);
tVec   = (0:N-1)'*dt;

% yaw: titta mot nästa hörn
yaw12 = atan2(P2(2)-P1(2), P2(1)-P1(1));
yaw23 = atan2(P3(2)-P2(2), P3(1)-P2(1));
yaw34 = atan2(P4(2)-P3(2), P4(1)-P3(1));
yaw41 = atan2(P1(2)-P4(2), P1(1)-P4(1));

nSeg = round(Tseg/dt);
kEnd = cumsum(nSeg);

yawRef = zeros(N,1);
yawRef(1:kEnd(1))                 = yaw12;  % P1->P2
yawRef(kEnd(1)+1:kEnd(2))         = yaw23;  % håll i P2, titta mot P3
yawRef(kEnd(2)+1:kEnd(3))         = yaw23;  % P2->P3
yawRef(kEnd(3)+1:kEnd(4))         = yaw34;  % håll i P3, titta mot P4
yawRef(kEnd(4)+1:kEnd(5))         = yaw34;  % P3->P4
yawRef(kEnd(5)+1:kEnd(6))         = yaw41;  % håll i P4, titta mot P1
yawRef(kEnd(6)+1:kEnd(7))         = yaw41;  % P4->P1
yawRef(kEnd(7)+1:kEnd(8))         = yaw12;  % håll i P1, titta mot P2

% Gör timeseries som Simulink gillar
x_ref   = timeseries(posRef(:,1), tVec);
y_ref   = timeseries(posRef(:,2), tVec);
z_ref   = timeseries(z0*ones(N,1), tVec);
yaw_ref = timeseries(yawRef,       tVec);

save('square_refs.mat','x_ref','y_ref','z_ref','yaw_ref');

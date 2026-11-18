close all
clear

pointsPosi=[0 0 0];
pointsHead=[0 0 0];

for i = 0:16            
    pointsPosi(i+1,:)=[0.5*cos(i*2*pi()/16) 0.5*sin(i*2*pi()/16) 0];
end

for i = 0:16            
    pointsHead(i+1,:)=[0 0 2*pi()*i/16+pi()];
end

pointsPosi=pointsPosi*roty(-1*pi()/4,'rad');

points=[pointsPosi pointsHead];

points=points+[0 0 1.5 0 0 0];



points=points+[0 4 0 0 0 0];

QDMAX=[ones(1,3)*0.8 ones(1,3)*0.5];
DT=0.2;
TACC=0.45;

traj = mstraj(points(2:end,:),QDMAX,[],points(1,:),DT,TACC);

figure
hold on
plot3(traj(:,1),traj(:,2),traj(:,3))
grid on
hold off

figure
hold on
axis([-0.75,0.75,3.25,4.75,0,2])
plot3(traj(:,1),traj(:,2),traj(:,3))
animate(SE3(traj(:,1:3))*SE3.rpy(traj(:,4:6)))
hold off
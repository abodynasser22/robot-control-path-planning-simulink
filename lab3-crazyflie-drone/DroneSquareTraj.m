close all
clear

square= [ 0.0  0.0;   % start
          0.5  0.0;
          0.5  0.5;
          0.0  0.5;
          0.0  0.0];  % tillbaka till start




QDMAX=[2 2];
DT=0.1;
TACC=0.9;

traj_square = mstraj(square(2:end,:),QDMAX,[],square(1,:),DT,TACC);


Gr_Six= [  0.0    0.0;   % 2b)
                           0.5    0.0;
                           0.5    -0.5;
                           0.0    -0.5;
                           0.0    0.5;
                           0.5    0.5;
                           0.5   0.25 ];



QDMAX=[2 2];
DT=0.1;
TACC=0.9;

traj_Six = mstraj(Gr_Six(2:end,:),QDMAX,[],Gr_Six(1,:),DT,TACC);




Sin= makeSinWaypoints2D(0, 5, 0.05, 0.3, 1.0, 0.0);


QDMAX=[2 2];
DT=0.1;
TACC=0.2;

traj_Sinusoidal = mstraj(Sin(2:end,:),QDMAX,[],Sin(1,:),DT,TACC);

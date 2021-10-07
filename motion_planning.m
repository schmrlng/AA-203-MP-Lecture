%% Problem 1 - Two Boxes
start = [0; 0];
goal = [.9 1 1 .9 .9; .9 .9 1 1 .9];
clearvars obstacles
obstacles{1} = [.1 .4 .4 .1 .1; .1 .1 .4 .4 .1];
obstacles{2} = [.4 .9 .9 .4 .4; .6 .6 .9 .9 .6];
clf; hold on;
axis square; rectangle;
fill(goal(1,:), goal(2,:), 'g')
for i = 1:length(obstacles)
    fill(obstacles{i}(1,:), obstacles{i}(2,:), 'r')
end
X_opt = [0 .4 .9 .9];
Y_opt = [0 .1 .6 .9];
c_opt = sum(fliplr(sqrt(diff(X_opt).^2 + diff(Y_opt).^2)))
plot(X_opt,Y_opt,'ob-','LineWidth',2,'MarkerFaceColor','b');

%% Problem 2 - Windy Maze
start = [.1; .65];
clearvars obstacles
obstacles{1} = [0 .9 .9 0 0; .45 .45 .55 .55 .45];
obstacles{2} = [.25 + [0 .05 .05 0 0]; .2 .2 .8 .8 .2];
obstacles{3} = [.75 + [0 .05 .05 0 0]; .2 .2 .8 .8 .2];
obstacles{4} = [.5 + [0 .05 .05 0 0]; .7 .7 1 1 .7];
obstacles{5} = [.5 + [0 .05 .05 0 0]; 0 0 .3 .3 0];
goal = [0 .15 .15 0 0; .3 .3 .45 .45 .3];
clf; hold on;
axis square; rectangle;
fill(goal(1,:), goal(2,:), 'g')
for i = 1:length(obstacles)
    fill(obstacles{i}(1,:), obstacles{i}(2,:), 'r')
end
X_opt = [.1 .25 .3 .5 .55 .75 .8 .9 .9 .8 .75 .55 .5 .3 .25 .15];
Y_opt = [.65 .8 .8 .7 .7 .8 .8 .55 .45 .2 .2 .3 .3 .2 .2 .3];
c_opt = sum(fliplr(sqrt(diff(X_opt).^2 + diff(Y_opt).^2)))
plot(X_opt,Y_opt,'ob-','LineWidth',2,'MarkerFaceColor','b');

%% RRT
figure(1)
Nmax = 1000;
tic
[c, N, ~] = RRT(start, goal, obstacles, Nmax, 1)
toc
title(sprintf('RRT c = %2f  N = %d', c/c_opt, N),'FontSize',18);

%% RRTstar
figure(2)
Nmax = 500;
tic
[c, N, ~] = RRTstar(start, goal, obstacles, Nmax, 1)
toc
title(sprintf('RRT* c = %2f  N = %d', c/c_opt, N),'FontSize',18);

%% PRMstar
figure(3)
Nmax = 500;
tic
[c, N, ~] = PRMstar(start, goal, obstacles, Nmax, 1.5, 1)
toc
title(sprintf('PRM* c = %2f  N = %d', c/c_opt, N),'FontSize',18);

%% FMTstar
figure(4)
Nmax = 500;
tic
[c, N, ~] = FMTstar(start, goal, obstacles, Nmax, 1.5, 1)
toc
title(sprintf('FMT* c = %2f  N = %d', c/c_opt, N),'FontSize',18);
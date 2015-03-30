function [c, N, A] = RRT(start, goal, obstacles, Nmax, FOR_THE_KIDDOS)

clf; hold on;
axis square; rectangle;
plot(start(1), start(2), 'sg')
fill(goal(1,:), goal(2,:), 'g')
for i = 1:length(obstacles)
    fill(obstacles{i}(1,:), obstacles{i}(2,:), 'r')
end

R = rand(2,Nmax);
V = zeros(2,Nmax);
V(:,1) = start;
N = 1;
A = zeros(1, Nmax);
eps = 0.1;   % maximum steering distance

for k = 1:Nmax
    q = R(:,k);
    [q_near, nn_idx] = NN1(V(:,1:N), q);
    q_new = q_near + (q - q_near)*min(eps/norm(q - q_near), 1);
    if check_segment(q_new, q_near, obstacles)
        N = N+1;
        V(:,N) = q_new;
        A(N) = nn_idx;
        plot([q_near(1), q_new(1)], [q_near(2), q_new(2)], 'ok-');
        if FOR_THE_KIDDOS
            drawnow;
        end
        if inpolygon(q_new(1), q_new(2), goal(1,:), goal(2,:))
            break
        end
    end
    if FOR_THE_KIDDOS == 2
        q_plot = plot([q_near(1), q_new(1), q(1)], [q_near(2), q_new(2), q(2)], 'ob--');
        pause;
        delete(q_plot);
    end
end

v = N;
X = V(1,v);
Y = V(2,v);
while v ~= 1
    v = A(v);
    X(end+1) = V(1,v);
    Y(end+1) = V(2,v);
end
plot(X,Y,'ob-','LineWidth',2,'MarkerFaceColor','b');
hold off;
c = sum(sqrt(diff(X).^2 + diff(Y).^2));
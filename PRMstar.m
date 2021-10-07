function [c, N, A] = PRMstar(start, goal, obstacles, Nmax, eta, FOR_THE_KIDDOS)

clf; hold on;
axis square; rectangle;
plot(start(1), start(2), 'sg')
fill(goal(1,:), goal(2,:), 'g')
mu = 1;
for i = 1:length(obstacles)
    fill(obstacles{i}(1,:), obstacles{i}(2,:), 'r')
    mu = mu - polyarea(obstacles{i}(1,:), obstacles{i}(2,:));
end

V = [start, rand(2,Nmax)];
for i = 1:length(obstacles)
    V = V(:,arrayfun(@(x,y) ~inpolygon(x, y, obstacles{i}(1,:), obstacles{i}(2,:)), V(1,:), V(2,:)));
end
N = size(V,2);
D = squareform(pdist(V'));

if FOR_THE_KIDDOS
    v_plot = plot(V(1,:),V(2,:),'ko');
    pause;
    delete(v_plot);
end

r = eta*sqrt(2*mu/pi*log(N)/N);     % 2 is 6 in the RRT* paper
friends = arrayfun(@(v) (v + find(D(v,v+1:end) < r)), 1:N, 'uniformoutput', false);
friends = cellfun(@(v,F) F(arrayfun(@(f) check_segment(V(:,v), V(:,f), obstacles), F)),...
                  num2cell(1:N), friends, 'uniformoutput', false);

cellfun(@(v,F) arrayfun(@(f) plot(V(1,[v, f]), V(2,[v, f]), 'ok-'), F), num2cell(1:N), friends);

A = cell2mat(cellfun(@(F) full(sparse(1, F, 1, 1, N)), friends, 'uniformoutput', false)');
G = D.*A;
G = sparse(max(G, G'));

goal_pts = find(arrayfun(@(x,y) inpolygon(x, y, goal(1,:), goal(2,:)), V(1,1:N), V(2,1:N)));
[C_all, P_all, ~] = graphshortestpath(G, 1);
C = C_all(goal_pts);
P = P_all(goal_pts);
[c, min_idx] = min(C);
if iscell(P)
    P = P{min_idx};
end
X = V(1,P);
Y = V(2,P);
plot(X,Y,'ob-','LineWidth',2,'MarkerFaceColor','b');
hold off;
% cumsum(fliplr(sqrt(diff(X).^2 + diff(Y).^2)))
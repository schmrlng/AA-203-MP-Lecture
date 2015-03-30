function [c, N, A] = FMTstar(start, goal, obstacles, Nmax, eta, FOR_THE_KIDDOS)

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
A = zeros(1, N);

plot(V(1,:),V(2,:),'ko');

r = eta*sqrt(2*mu/pi*log(N)/N);
NN = arrayfun(@(v) [find(D(v,1:v-1) < r), v + find(D(v,v+1:end) < r)], 1:N, 'uniformoutput', false);
W = true(1,N);
W(1) = false;
H = false(1,N);
H(1) = true;
C = inf*ones(1,N);
C(1) = 0;
z = 1;

while ~inpolygon(V(1,z), V(2,z), goal(1,:), goal(2,:))
    H_new = [];
    for x = NN{z}(W(NN{z}))
        Y_near = NN{x}(H(NN{x}));
        [c_min, y_idx] = min(C(Y_near) + D(x, Y_near));
        y_min = Y_near(y_idx);
        if check_segment(V(:,x), V(:,y_min), obstacles)
            A(x) = y_min;
            C(x) = c_min;
            plot(V(1,[x, y_min]), V(2,[x, y_min]), 'ok-')
            if FOR_THE_KIDDOS
                drawnow;
            end
            H_new(end+1) = x;
            W(x) = false;
        end
    end
    H(H_new) = true;
    H(z) = false;
    z = find(C == min(C(H)));   % shitty
end

v = z;
X = V(1,v);
Y = V(2,v);
while v ~= 1
    v = A(v);
    X(end+1) = V(1,v);
    Y(end+1) = V(2,v);
end
plot(X,Y,'ob-','LineWidth',2,'MarkerFaceColor','b');
hold off;
c = C(z);
% cumsum(fliplr(sqrt(diff(X).^2 + diff(Y).^2)))
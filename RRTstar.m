function [c, N, A] = RRTstar(start, goal, obstacles, Nmax, FOR_THE_KIDDOS)

clf; hold on;
axis square; rectangle;
plot(start(1), start(2), 'sg')
fill(goal(1,:), goal(2,:), 'g')
mu = 1;
for i = 1:length(obstacles)
    fill(obstacles{i}(1,:), obstacles{i}(2,:), 'r')
    mu = mu - polyarea(obstacles{i}(1,:), obstacles{i}(2,:));
end

R = rand(2,Nmax);
V = zeros(2,Nmax);
C = zeros(1,Nmax);
V(:,1) = start;
C(1) = 0;
N = 1;
A = zeros(1,Nmax);
eta = 0.1;
r = @(N) min(1.1*sqrt(3*mu/pi*log(N)/N), eta);

if FOR_THE_KIDDOS
    axis manual;
    H = zeros(1,Nmax);
    circle = @(x,y,r) plot(x+r*cos(linspace(0,2*pi,20)),y+r*sin(linspace(0,2*pi,20)),'b-');
end

for k = 1:Nmax
    q = R(:,k);
    [q_nrst, nrst_idx] = NN1(V(:,1:N), q);
    q_new = q_nrst + (q - q_nrst)*min(eta/norm(q - q_nrst), 1);
%     if FOR_THE_KIDDOS == 3
%         q_plot = plot([q_nrst(1), q_new(1), q(1)], [q_nrst(2), q_new(2), q(2)], 'ob--');
%         pause;
%         delete(q_plot);
%     end
    if check_segment(q_new, q_nrst, obstacles)
        [near_idx, near_norms] = NNr(V(:,1:N), q_new, r(N));
        N = N+1;
        V(:,N) = q_new;
        min_idx = nrst_idx;
        min_c = C(nrst_idx) + norm(q_new - q_nrst);
        [costs, c_idxidx] = sort(near_norms + C(near_idx));
        for i = 1:length(costs)
            if costs(i) >= min_c
                break
            end
            if check_segment(q_new, V(:,near_idx(c_idxidx(i))), obstacles)
                min_idx = near_idx(c_idxidx(i));
                min_c = costs(i);
                break
            end
        end
        A(N) = min_idx;
        C(N) = min_c;
        if FOR_THE_KIDDOS
            H(N) = plot(V(1,[min_idx, N]), V(2,[min_idx, N]), 'ok-');
            drawnow;
            if FOR_THE_KIDDOS == 2
                h = circle(V(1,N),V(2,N),r(N));
                pause;
            end
        end
        rw_idx = find(C(N) + near_norms < C(near_idx));
        for i = rw_idx
            if check_segment(q_new, V(:,near_idx(i)), obstacles)
                if FOR_THE_KIDDOS
                    delete(H(near_idx(i)));
                    H(near_idx(i)) = plot(V(1,[N,near_idx(i)]), V(2,[N,near_idx(i)]), 'ok-');
%                     drawnow;
                end
                A(near_idx(i)) = N;
                c_delta = C(N) + near_norms(i) - C(near_idx(i));
                progeny = near_idx(i);
                while ~isempty(progeny)
                    C(progeny) = C(progeny) + c_delta;
                    progeny = find(ismember(A, progeny));
                end
            end
        end
        if FOR_THE_KIDDOS == 2
            delete(h)
        end
    end
end

if ~FOR_THE_KIDDOS
    for v = 2:N
        plot(V(1,[A(v), v]), V(2,[A(v), v]), 'ok-');
    end
end

goal_pts = find(arrayfun(@(x,y) inpolygon(x, y, goal(1,:), goal(2,:)), V(1,1:N), V(2,1:N)));
[c, min_idx] = min(C(goal_pts));
v = goal_pts(min_idx);
X = V(1,v);
Y = V(2,v);
while v ~= 1
    v = A(v);
    X(end+1) = V(1,v);
    Y(end+1) = V(2,v);
end
plot(X,Y,'ob-','LineWidth',2,'MarkerFaceColor','b');
hold off;
% cumsum(fliplr(sqrt(diff(X).^2 + diff(Y).^2)))
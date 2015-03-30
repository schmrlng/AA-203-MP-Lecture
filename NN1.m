function [nearest, nn_idx] = NN1(V, q)

[~, nn_idx] = min(sum((V-q*ones(1,size(V,2))).^2,1));
nearest = V(:,nn_idx);
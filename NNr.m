function [nn_idx, norms] = NNr(V, q, r)

d2 = sum((V-q*ones(1,size(V,2))).^2,1);
nn_idx = find(d2 < r^2);
norms = sqrt(d2(nn_idx));
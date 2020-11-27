function [ X ] = pattern_invariance( S )
% Generate a maximally sparsity-wise invariant (MSI) subplace with respect to X
% See Section IV of the following paper
% "On Separable Quadratic Lyapunov Functions for Convex Design of Distributed Controllers"

m = size(S,1);
n = size(S,2);
X = ones(n,n);

% Analytical solution with complexity mn^2
for i = 1:m
        for k = 1:n
                if S(i,k)==0
                        for j = 1:n
                                if S(i,j) == 1
                                        X(j,k) = 0;
                                end
                        end
                end
        end
end

% symmetric part
Xu = triu(X').*triu(X);
X  = Xu + Xu';
X  = full(spones(X));

end


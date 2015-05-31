function [K, f_length, fval] = estimate_K_ax(P)
syms x real
K=[x,0,0;0,x,0;0,0,1];
I=eye(3);

%find D
D=inv(K)*P*K*(inv(K)*P*K)'-I;

%||vec(D)||
[r,c] = size(D);
vec = reshape(D,r*c,1);
n = norm(vec,1);
ht = matlabFunction(n);

%plot
%ezplot(ht);

%% min ax & ay

% PSO algorithm
% rng default;
% options = optimoptions('particleswarm','SwarmSize',100);
% options.HybridFcn = @fmincon;
% lb = [-30000;-30000];
% ub = -lb;
% [f_length,fval] = particleswarm(ht,1,lb,ub,options);

% Multidimensional unconstrained nonlinear minimization
v = [2000];
[f_length,fval] = fminsearch(ht,v);

K=[f_length,0,0;0,f_length,0;0,0,1];
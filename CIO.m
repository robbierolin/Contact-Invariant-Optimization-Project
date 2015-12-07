%% Declare starting conditions.
[ N,K,PhaseLength,deltaT,T ] = getConstants();

s0 = getS0(K, N, PhaseLength);

%% Run Optimization.
% maxFunEvals = 25;
% options = [];
% options.displau = 'none';
% options.maxFunEvals = maxFunEvals;
% options.Method = 'lbfgs';
% cd minFunc_2012;
% addpath(genpath(pwd));
% cd ..;
% s = minFunc(@L, s, options);
options = optimoptions(@fminunc, 'MaxFunEvals', 10000);
% TODO: Compute gradient.
s = fminunc(@OptimizationPhase1, s0, options);
Visualize(s,K,N);
% s = perturb(s);
% s = fminunc(@OptimizationPhase2, s, options);
% Visualize(s,K,N); 
% s = perturb(s);
% s = fminunc(@OptimizationPhase3, s, options);


%% Interpret Results.

% Visualize(s, K, N);



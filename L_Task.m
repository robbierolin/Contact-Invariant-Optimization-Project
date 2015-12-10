 function [ cost,g ] = L_Task( s, f, u, q_dotdot )
%L_TASK Summary of this function goes here
%   Detailed explanation goes here
%% Declare constants.
[ N,K,PhaseLength,deltaT,T ] = getConstants();

%% L_Task - Smoothness term.
smoothnessTerm = 0;
% for t=1:T
%    f_t = f(t, :);
%    u_t = u(t, :);
%    q_t = q_dotdot(t, :);
%    smoothnessTerm = smoothnessTerm + norm(f_t)^2 + norm(u_t)^2 + norm(q_t)^2;
% end


%% L_Task - task term.

% Desired features:
lhp = [-1 0 1];
rhp = [1 0 1];
lho = [0 -1 0];
rho = [0 -1 0];
lfp = [-1 5.3 1];
rfp = [1 5.3 1];
lfo = [0 1 0];
rfo = [0 1 0];
tp = [0 2.3 1];
to = [0 0 -1];
h_star = [lhp rhp lho rho lfp rfp lfo rfo tp to];
s_order = [tp to lfp rfp lhp rhp lfo rfo lho rho];
% Actual features.
[tp,to,lfp,lfo,rfp,rfo,lhp,lho,rhp,rho] = getBodyPositions( s, K, N );
h = [lhp(:,K)' rhp(:,K)' lho(:,K)' rho(:,K)' lfp(:,K)' rfp(:,K)' lfo(:,K)' rfo(:,K)' tp(:,K)' to(:,K)'];
taskTerm = norm(h - h_star)^2;
taskScale = 10^3;
taskTerm = taskTerm * taskScale;

fprintf('\ntaskTerm: %f\n', taskTerm);
fprintf('smoothnessTerm: %f\n', smoothnessTerm);
cost = taskTerm + smoothnessTerm/1000;

%% Calculate gradient
% In negative direction of task, more weight for times closer to T
% g = zeros(size(s));
% s0 = getS0();
% sizek = 30;
% startPos = s0(1:sizek);
% for k = 1:K
%     % Get xk
% 
%     xk = s((k-1)*sizek + 1 : k*sizek);
% 
%     % Compute difference between xk and final task
%     diff = xk - ((k-1)/(K-1) * s_order + (K-k)/(K-1) * startPos);
%     
%     % Gradient is difference
%     g((k-1)*sizek + 1 : k*sizek) = diff; 
% end
end


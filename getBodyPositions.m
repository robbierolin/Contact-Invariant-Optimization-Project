function [tp,to,lfp,lfo,rfp,rfo,lhp,lho,rhp,rho] = getBodyPositions( s, K, N )
%GETBODYPOSITIONS Summary of this function goes here
%   Detailed explanation goes here

x1K = s(1:K*6*(N+1));

tp = zeros(3,K); % Torso position.
to = zeros(3,K); % Torso orientation.
lfp = zeros(3,K); % Left foot position.
lfo = zeros(3,K); % Left foot orientation.
rfp = zeros(3,K); % Right foot position.
rfo = zeros(3,K); % Right foot orientation.
lhp = zeros(3,K); % Left hand position.
lho = zeros(3,K); % Left hand orientation.
rhp = zeros(3,K); % Right hand position.
rho = zeros(3,K); % Right hand orientation.
for k=1:K
   % position and orientation of all end-effectors at phase k
   x_k = x1K(((k-1)*6*(N+1) + 1) : k*6*(N+1));
   tp(:,k) = x_k(1:3)';
   to(:,k) = x_k(4:6)';
   lfp(:,k) = x_k(7:9)';
   rfp(:,k) = x_k(10:12)';
   lhp(:,k) = x_k(13:15)';
   rhp(:,k) = x_k(16:18)';
   lfo(:,k) = x_k(19:21)';
   rfo(:,k) = x_k(22:24)';
   lho(:,k) = x_k(25:27)';
   rho(:,k) = x_k(28:30)';
end

end


function [ s0 ] = getS0(K,N,PhaseLength)
%GETS0 Returns the starting value of the solution vector s
%   Detailed explanation goes here

% s is a vector with dimension K(12(N+1)+N) and is of the form
% [x_1 ... x_k x'_1 ... x'_k c_1 ... c_k]
% and x is of the form
% [p_c r_c p_1 ... p_N r_1 ... r_N]

% Starting position (standing at (0,0,0) ): 
p_c = [0 3 0]; % Torso position.
r_c = [0 0 1]; % Torso orientation. (normal)

p_1 = [-1 0 0]; % Left foot position.
r_1 = [0 -1 0]; % Left foot orientation.

p_2 = [1 0 0]; % Right foot position.
r_2 = [0 -1 0]; % Right foot orientation.

p_3 = [-2 2 0]; % Left hand position.
r_3 = [0 -1 0]; % Left hand orientation.

p_4 = [2 2 0]; % Right hand position.
r_4 = [0 -1 0]; % Right hand orientation.

x_1 = [p_c r_c p_1 p_2 p_3 p_4 r_1 r_2 r_3 r_4];

% End position (hand stand centred at (0,0,1) ) :
% p_c = [0 2.3 1]; % Torso position.
% r_c = [0 0 -1]; % Torso orientation. (normal)
% 
% p_1 = [-1 5.3 1]; % Left foot position.
% r_1 = [0 1 0]; % Left foot orientation.
% 
% p_2 = [1 5.3 1]; % Right foot position.
% r_2 = [0 1 0]; % Right foot orientation.
% 
% p_3 = [-1 0 1]; % Left hand position.
% r_3 = [0 -1 0]; % Left hand orientation.
% 
% p_4 = [1 0 1]; % Right hand position.
% r_4 = [0 -1 0]; % Right hand orientation.

% End position - same as start position.
p_c = [0 3 0]; % Torso position.
r_c = [0 0 1]; % Torso orientation. (normal)

p_1 = [-1 0 0]; % Left foot position.
r_1 = [0 -1 0]; % Left foot orientation.

p_2 = [1 0 0]; % Right foot position.
r_2 = [0 -1 0]; % Right foot orientation.

p_3 = [-2 2 0]; % Left hand position.
r_3 = [0 -1 0]; % Left hand orientation.

p_4 = [2 2 0]; % Right hand position.
r_4 = [0 -1 0]; % Right hand orientation.

x_K = [p_c r_c p_1 p_2 p_3 p_4 r_1 r_2 r_3 r_4];
x = x_1;
for k=2:K
   x_k = x_1 + (x_K - x_1) * (k-1)/(K-1);
   x = horzcat(x, x_k);
end

% Create x' approximated by finite difference from end to start.
xdot_avg = (x_K - x_1) / (K*PhaseLength);
xdot = repmat(xdot_avg, 1, K);

c_n1k1 = 1; % Left foot starts on floor.
c_n2k1 = 1; % Right foot starts on floor.
c_n3k1 = 0; % Left hand does not start on floor.
c_n4k1 = 0; % Right hand does not start on floor.
c_1 = [c_n1k1 c_n2k1 c_n3k1 c_n4k1];
% c_n1kK = 0; % Left foot does not end on floor.
% c_n2kK = 0; % Right foot does not end on floot.
% c_n3kK = 1; % Left hand ends on the floor.
% c_n4kK = 1; % Right hand ends on the floor.

% Same start and end pose. 
c_n1kK = 1; % Left foot does not end on floor.
c_n2kK = 1; % Right foot does not end on floot.
c_n3kK = 0; % Left hand ends on the floor.
c_n4kK = 0; % Right hand ends on the floor.
c_K = [c_n1kK c_n2kK c_n3kK c_n4kK];

% Assume no contacts active any time between (can assume all contacts
% active).
c_mid = zeros(1, N*(K-2));
c = [c_1 c_mid c_K];


% s0 = [x xdot c];
s0 = [x c];
end


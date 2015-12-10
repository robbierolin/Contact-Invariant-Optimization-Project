function [ cost, F, u, pose_dotdot ] = L_Physics(s)
%L_PHYSICS Summary of this function goes here
%   Detailed explanation goes here

%% Declare constants.
[ N,K,PhaseLength,deltaT,T ] = getConstants();

%% Calculate L_Physics.
cost = 0;
D = 6*(N+1); % DOF, 6 per end-effector and torso
pose = q(s,N,T,K,D);
BD = diag(ones(T,1)) - diag(ones(T-1,1),-1); % Backwards difference operator.
BD(1,1) = 0;
pose_dot = (1/deltaT) * BD * pose;
pose_dotdot = (1/deltaT) * BD * pose_dot;
F = zeros(T,6*N);
u = zeros(T,6*N);


J = horzcat(zeros(6*N,6), eye(6*N));
B = J';
M = zeros(D,D); % Inertia matrix.
C = zeros(D,D); % Matrix of Coriolis.

g = zeros(D,1); % Gravity. TODO: make force depend on character position.
g(2) = 10 * -9.81; % Downward force on each end effector.

R = eye(6*N); % TODO: something else.

M = [4 4 4 2 2 2 2 2 2 1 1 1 1 1 1 4 4 4 2 2 2 2 2 2 1 1 1 1 1 1];

for t = 1:T
   % Inverse dynamics are computed by getting Tao(s), J(s), A(s), b(s), W(s), 
   % and solving for f(s) and u(s) to compute physics-violation cost.
%    J = eye(6*N, D); % TODO: figure out what this should do. Map contact forces on each end effector to forces for each degree of freedom.
%    
%    B = eye(D,6*N); % TODO: map applied forces to full space.
%    B = J';
   
   qt = pose(t, :);
   torsoPosition = qt(1:3);
%    for i=1:N
%       deltaP = qt(3*i+1 : 3*(i+1)) - torsoPosition;
%       if i == 1 || i == 2
%           mass = 2;
%       else
%           mass = 1;
%       end
%       deltaP = deltaP * mass;
%       deltaPMat = [0 -deltaP(3) deltaP(2); deltaP(3) 0 -deltaP(1); -deltaP(2) deltaP(1) 0];
%       M(3*i+1:3*(i+1), 3*i+1:3*(i+1)) = deltaPMat(:,:)^2;
%    end
   
   q_dot = pose_dot(t,:);

%    for i = 1:D
%        for j = 1:D
%           C(i,j) = sum(cijk(i,j,1:D,M).*q_dot(1:D));
%        end 
%    end
   

%    g(5) = 2 * -9.81;
%    g(8) = 2 * -9.81;
%    g(11) = -9.81;
%    g(14) = -9.81;
   
   q_dotdot = pose_dotdot(t,:);

   contact_forces = zeros(D,1);
   effectors_on_ground = 0;
   for i = 2:3:14
       if qt(i) < 0.1 % if end effector is on ground
           contact_forces(i) =  10 * 9.8 - q_dotdot(i) * M(i); % Add normal force plus impact force
           effectors_on_ground = effectors_on_ground + 1;
       end
   end
   if effectors_on_ground > 0
       contact_forces = contact_forces / effectors_on_ground;
   end
   applied_forces = q_dotdot .* M;
   
%    tao = M*q_dotdot' + C*q_dot' + g;
    tao = contact_forces + applied_forces';
  
   A = zeros(N,6*N);
   
   b = 10000*ones(N,1); % Normal force for each end effector.
   for i = 1:N
      if qt(3*i+2) < 0.1 % If y component of end effector is close to 0.
          if i == 1 || i == 2
              b(i) = 2 * 9.81;
          end
          A(i, 6*(i-1)+1) = 1;
          A(i, 6*(i-1)+3) = 1;
      end
   end
   
   
   W = zeros(6*N);
   wIndex = 1;
   for i=1:N
       for j=1:6
           c_i = c(s, i, phi(t,K,T),K,N);
           k_0 = 10^-2;
           k_1 = 10^-3;
           W(wIndex,wIndex) = k_0 / (c_i + k_1);
           wIndex = wIndex + 1;
       end 
   end
   
  
   
   % Run quadratic program solver to get f, u.
   H = zeros(12*N + 1);
   H(1:6*N,1:6*N) = W;
   H(6*N+1:12*N, 6*N+1:12*N) = R;
   f = zeros(12*N + 1, 1);
   Jt = J';
   for i = 1:6*N
      f(i) = sum(abs(Jt(:,i)));
   end
   for i = 6*N+1:12*N
      f(i) = sum(abs(B(:, i-(6*N)))); 
   end
   f(12*N + 1) = -1*sum(abs(tao));
   
   options = optimoptions('quadprog',...
    'Algorithm','interior-point-convex','Display','off');
   A = horzcat(A, zeros(N,6*N+1));
   Aeq = zeros(1, 12*N+1);
   Aeq(12*N+1) = 1;
   beq = 1;
   x = quadprog(H,f,A,b,Aeq,beq,[],[],[],[],options);
   x = x/x(12*N +1);
   f_t = x(1:6*N);
   u_t = x(6*N+1:12*N);
   
   cost = cost + (norm(Jt*f_t + B*u_t - tao))^2;

   F(t,:) = f_t(:);
   u(t,:) = u_t(:);
end

%% Calculate gradient

end


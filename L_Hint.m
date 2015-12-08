function [ cost ] = L_Hint( s )
%L_HINT Summary of this function goes here
%   Detailed explanation goes here
[ N,K,PhaseLength,deltaT,T ] = getConstants();
eps = 0.1;

% Get spline of torso position and accelerations
D = 6*(N+1); % DOF, 6 per end-effector and torso
pose = q(s,N,T,K,D);
BD = diag(ones(T,1)) - diag(ones(T-1,1),-1); % Backwards difference operator.
BD(1,1) = 0;
pose_dot = (1/deltaT) * BD * pose;
pose_dotdot = (1/deltaT) * BD * pose_dot;

cost = 0;
for t = 1:T
    %% compute z - zero moment point
    qt = pose(t,:);
    qdt = pose_dot(t,:);
    qddt = pose_dotdot(t,:);
    p = qt(1:3); % torso position at time t.
    px = p(1);
    py = p(2);
    pz = p(3);
    pdd = qddt(1:3); % torso acceleration at time t.
    pddx = pdd(1);
    pddy = pdd(2);
    pddz = pdd(3);
    w = qdt(16:18); % torso angular velocity at time t.
    wx = w(1);
    wy = w(2);
    wz = w(3);
    mass = 10;
    gy = -9.81;
    gx = 0;
    gz = 0;

    ix = mass/12 * (1 + 4); % wikipedia/list_of_moments_of_intertia
    iz = mass/12 * (4 + 4);

    x_zmp = (mass * (px * (pddy + gy) - py * (pddx + gx)) - iz*wz) / (mass * (pddy + gy));


    z_zmp = (mass * (pz * (pddy + gy) - py * (pddz + gz)) - ix*wx) / (mass * (pddy + gy));

    z = [x_zmp 0 z_zmp];

    %% compute n - nearest point to z in the convex hull
    % Set up lp to get weights
    lb = [0 0 0 0];
    Aeq = [1 1 1 1];
    beq = 1;
    d = z;
    C = zeros(3,4);
    for i = 1:N
       % Get position of each end effector
       C(:, i) = qt((3*i)+1 : 3*(i+1));
    end
    lambda = lsqlin(C,d,[],[],Aeq,beq,lb,[]);
    
    % Add each end-effector multiplied by its weight to get n
    n = zeros(3,1);
    for i = 1:N
       n = n + lambda(i) * C(:,i); 
    end
    %% Add cost
    cost = cost + max(norm(z - n') - eps, 0)^2;
end

%% Compute gradient.
end


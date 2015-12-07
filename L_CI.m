function [ cost ] = L_CI(s)
%L_CI Summary of this function goes here
%   Detailed explanation goes here

%% Declare constants.
[N,K,PhaseLength,deltaT,T] = getConstants();


%% Calculate L_CI.
e = zeros(N,T,4);
[tp,to,lfp,lfo,rfp,rfo,lhp,lho,rhp,rho] = getBodyPositions( s, K, N );
end_effector_positions = zeros(4, 3, K);
end_effector_positions(1, :, :) = lfp(:, :);
end_effector_positions(2, :, :) = rfp(:, :);
end_effector_positions(3, :, :) = lhp(:, :);
end_effector_positions(4, :, :) = rhp(:, :);

end_effector_orientations = zeros(4, 3, K);
end_effector_orientations(1, :, :) = lfo(:, :);
end_effector_orientations(2, :, :) = lho(:, :);
end_effector_orientations(3, :, :) = rfo(:, :);
end_effector_orientations(4, :, :) = rho(:, :);

for i = 1:N
    % Get position of end-effector i over time.
    end_effector_position = end_effector_positions(i, :, :);
    end_effector_position = reshape(end_effector_position, 3, K);       
    pos_spline = cscvn(end_effector_position);
    
    % Get orientation of end-effector i over time
    end_effector_orientation = end_effector_orientations(i, :, :);
    end_effector_orientation = reshape(end_effector_orientation, 3, K);
    orientation_spline = cscvn(end_effector_orientation);
    for t = 1:T
       % Get position of end effector i at time t
       value = fnval(pos_spline, ((K*t/T)-1)*pos_spline.breaks(2));
       
       % Get difference between value and closest point in environment.
       % TODO: also consider other body parts.
       % TODO: use virtual min operator.
       difference_vector = [0 value(2) 0];
       
       % Get difference in surface normals.
       normal = fnval(orientation_spline, ((K*t/T)-1)*orientation_spline.breaks(2));
       difference_angle = acos(normal(2) / norm(normal));
       
       
       e(i,t,:) = [difference_vector difference_angle];
    end
    
end

% TODO: Calculate eDot

% Get c, sum values.
cost = 0;
for t = 1:T
    for i=1:N
       contact = c(s, i, phi(t, K, T), K, N);
       normValue = e(i,t,1)*e(i,t,1) + e(i,t,2)*e(i,t,2) + e(i,t,3)*e(i,t,3) + e(i,t,4)*e(i,t,4);
       cost = cost + contact*normValue;
    end
end

end


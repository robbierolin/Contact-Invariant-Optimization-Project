function [] = Visualize( s, K, N)
%VISUALIZE Summary of this function goes here
%   Detailed explanation goes here

% Get the position (and orientation) of each end-effector over K time
% steps. Create spline and graph.

[tp,to,lfp,lfo,rfp,rfo,lhp,lho,rhp,rho] = getBodyPositions( s, K, N );

% For visualization purposes, swap y and z so that y corresponds to the
% vertical axis.
tp([2 3], :) = tp([3 2], :);
to([2 3], :) = to([3 2], :);
lfp([2 3], :) = lfp([3 2], :);
rfp([2 3], :) = rfp([3 2], :);
lhp([2 3], :) = lhp([3 2], :);
rhp([2 3], :) = rhp([3 2], :);
lfo([2 3], :) = lfo([3 2], :);
rfo([2 3], :) = rfo([3 2], :);
lho([2 3], :) = lho([3 2], :);
rho([2 3], :) = rho([3 2], :);

hold on;
% Plot start and end points.
text(tp(1, 1), tp(2, 1), tp(3, 1), 'Torso start');
text(tp(1, K), tp(2, K), tp(3, K), 'Torso end');

text(lfp(1, 1), lfp(2, 1), lfp(3, 1), 'Left foot start');
text(lfp(1, K), lfp(2, K), lfp(3, K), 'Left foot end');

text(rfp(1, 1), rfp(2, 1), rfp(3, 1), 'Right foot start');
text(rfp(1, K), rfp(2, K), rfp(3, K), 'Right foot end');

text(lhp(1, 1), lhp(2, 1), lhp(3, 1), 'Left hand start');
text(lhp(1, K), lhp(2, K), lhp(3, K), 'Left hand end');

text(rhp(1, 1), rhp(2, 1), rhp(3, 1), 'Right hand start');
text(rhp(1, K), rhp(2, K), rhp(3, K), 'Right hand end');

% Plot torso and end effectors over time.
% Torso: 
plot3(tp(1,:), tp(2,:), tp(3,:), 'o');
fnplt(cscvn(tp))

% Left foot:
plot3(lfp(1, :), lfp(2, :), lfp(3, :), 'o')
fnplt(cscvn(lfp));

% Right foot:
plot3(rfp(1, :), rfp(2, :), rfp(3, :), 'o')
fnplt(cscvn(rfp));

% Left hand:
plot3(lhp(1, :), lhp(2, :), lhp(3, :), 'o')
fnplt(cscvn(lhp));

% Right hand:
plot3(rhp(1, :), rhp(2, :), rhp(3, :), 'o')
fnplt(cscvn(rhp));

% Plot normals.
% for k = 1:K
%    rh_normal = rho(:,k) / norm(rho(:,k));
%    line([rhp(1,k) rhp(1,k)+rh_normal(1)], [rhp(2,k) rhp(2,k)+rh_normal(2)], [rhp(3,k) rhp(3,k)+rh_normal(3)]); 
%    
%    lh_normal = lho(:,k) / norm(lho(:,k));
%    line([lhp(1,k) lhp(1,k)+lh_normal(1)], [lhp(2,k) lhp(2,k)+lh_normal(2)], [lhp(3,k) lhp(3,k)+lh_normal(3)]); 
%    
%    rf_normal = rfo(:,k) / norm(rfo(:,k));
%    line([rfp(1,k) rfp(1,k)+rf_normal(1)], [rfp(2,k) rfp(2,k)+rf_normal(2)], [rfp(3,k) rfp(3,k)+rf_normal(3)]); 
%    
%    lf_normal = lfo(:,k) / norm(lfo(:,k));
%    line([lfp(1,k) lfp(1,k)+lf_normal(1)], [lfp(2,k) lfp(2,k)+lf_normal(2)], [lfp(3,k) lfp(3,k)+lf_normal(3)]); 
%    
%    t_normal = to(:,k) / norm(to(:,k));
%    line([tp(1,k) tp(1,k)+t_normal(1)], [tp(2,k) tp(2,k)+t_normal(2)], [tp(3,k) tp(3,k)+t_normal(3)]); 
% end

% Plot stick figure - starting position.
hold on 
line([-1 -1], [0 0], [0 2]);
line([1 1], [0 0], [0 2]);
line([-1 1], [0 0], [2 2]);
line([-1 1], [0 0], [4 4]);
line([-1 -1], [0 0], [2 4]);
line([1 1], [0 0], [2 4]);
line([-2 -1], [0 0], [2 3.5]);
line([1 2], [0 0], [3.5 2]);
line([-0.5 -0.5], [0 0], [4 5]);
line([0.5 0.5], [0 0], [4 5]);
line([-0.5 0.5], [0 0], [5 5]);
% 
% % Plot stick figure - end position.
hold on
line([-1 -1], [1 1], [0 1.8]);
line([1 1], [1 1], [0 1.8]);
line([-1 -1], [1 1], [1.3 3.3]);
line([1 1], [1 1], [1.3 3.3]);
line([-1 1], [1 1], [1.3 1.3]);
line([-1 1], [1 1], [3.3 3.3]);
line([-1 -1], [1 1], [3.3 5.3]);
line([1 1], [1 1], [3.3 5.3]);
line([-0.5 -0.5], [1 1], [0.3 1.3]);
line([0.5 0.5], [1 1], [0.3 1.3]);
line([-0.5 0.5], [1 1], [0.3 0.3])
end


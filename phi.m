function [ index ] = phi(t,K,T)
%PHI For a given time step t from 1 <= t <= T, return which phase t is in, 
% given that T is partitioned into K phases.
index = ceil(t/T * K);
end


function [ c ] = c( s, i, phi, K, N )
%C Summary of this function goes here
%   Detailed explanation goes here

sizeX = 6*(N+1);
% c = s(sizeX * K * 2 + (phi - 1) * N + i);
c = s(sizeX * K  + (phi - 1) * N + i);

end


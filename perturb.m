function [ s ] = perturb( s )
%PERTURB Summary of this function goes here
%   Detailed explanation goes here
length = size(s,2);
for i=1:length
   s(i) = s(i) + normrnd(0,0.1);
end

end


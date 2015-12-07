function [ c ] = cijk(i,j,k,M)
%CIJK Summary of this function goes here
%   Detailed explanation goes here
D = size(M,1);
if i == D | j == D | k == D
    c = 0;
    return;
end

if k == i
    partial_MijByQk = M(i+1,j) - M(i,j);
elseif k == j
    partial_MijByQk = M(i,j+1) - M(i,j);
else
    partial_MijByQk = 0;
end

if j == i
    partial_MikByQj = M(i+1,k) - M(i,k);
elseif j == k
    partial_MikByQj = M(i,k+1) - M(i,k);
else
    partial_MikByQj = 0;
end

if i == j
    partial_MjkByQi = M(j+1,k) - M(j,k);
elseif i == k
    partial_MjkByQi = M(j,k+1) - M(j,k);
else
    partial_MjkByQi = 0;
end

    

c = .5 * (partial_MijByQk + partial_MikByQj - partial_MjkByQi);  

end


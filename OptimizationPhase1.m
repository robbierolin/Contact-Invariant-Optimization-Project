function [ cost ] = OptimizationPhase1( s )
%OPTIMIZATIONPHASE1 Summary of this function goes here
%   Detailed explanation goes here
[cost, f, u, pose_dotdot ] = L_Physics(s);
cost = L_Task(s, f, u, pose_dotdot);
end


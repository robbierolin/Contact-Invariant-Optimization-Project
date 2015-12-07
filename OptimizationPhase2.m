function [ cost ] = OptimizationPhase2( s )
%OPTIMIZATIONPHASE2 Summary of this function goes here
%   Detailed explanation goes here

lci = L_CI(s);
[lphysics,f, u, pose_dotdot] = L_Physics(s);
ltask = L_Task(s,f,u,pose_dotdot);
cost = lci + ltask + 0.1*lphysics;

end


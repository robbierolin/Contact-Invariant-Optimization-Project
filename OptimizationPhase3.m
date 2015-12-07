function [ cost ] = OptimizationPhase3(s)
%OPTIMIZATIONPHASE3 Summary of this function goes here
%   Detailed explanation goes here
lci= L_CI(s);
[lphysics,f, u, pose_dotdot] = L_Physics(s);
ltask = L_Task(s,f,u,pose_dotdot);
cost = lci + lphysics + ltask;

end


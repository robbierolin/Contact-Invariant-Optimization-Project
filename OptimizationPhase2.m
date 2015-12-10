function [ cost ] = OptimizationPhase2( s )
%OPTIMIZATIONPHASE2 Summary of this function goes here
%   Detailed explanation goes here

lci = L_CI(s);
[lphysics,f, u, pose_dotdot] = L_Physics(s);
ltask = L_Task(s,f,u,pose_dotdot);
lhint = L_Hint(s);
lphysics = 0.000001*lphysics
lhint
lci
ltask
cost = lci + ltask + lphysics + lhint;

end


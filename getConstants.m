function [ N,K,PhaseLength,deltaT,T ] = getConstants()
%GETCONSTANTS Summary of this function goes here
%   Detailed explanation goes here

K = 3; % Number of intervals/phases.
PhaseLength = 0.1; % Length of phase (seconds).
deltaT = 0.1; % Time interval for evaluating inverse dynamics and costs. 
T = round(PhaseLength*K / deltaT); % Number of discrete time steps.
N = 4; % Number of 'end-effectors'. (limbs)
end


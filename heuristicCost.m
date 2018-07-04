function [ c ] = heuristicCost( g )
%heuristic cost of a vc configuration
w=-7;
c=w*g(2,3);%only cares y displacement
% c=-w*atan2(g(2,1),g(1,1));
end


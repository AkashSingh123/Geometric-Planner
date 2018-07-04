function [ ad ] = myAdjoint( g )
%compute adjoint opteration from g
ad=[g(1:2,1:2),[g(2,3);-g(1,3)];0 0 1];
end


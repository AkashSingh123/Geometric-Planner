function [ w ] = forceMapping( g )
%returns a transformation which transfrom the force in a module frame to
%the head module frame
w=[g(1:2,1:2),[0;0];-g(2,3),g(1,3),1];

end


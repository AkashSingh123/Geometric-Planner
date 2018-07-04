function [ h ] = xiHat( xi )
h=[0,-xi(3),xi(1);xi(3),0,xi(2);0,0,0];
end


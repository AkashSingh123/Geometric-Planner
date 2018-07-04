function [ com ] = computeCOM( g )
n=size(g,2);
com=zeros(2,1);
for i=1:n
    com=com+g{i}(1:2,3);
end
com=com/n;

end


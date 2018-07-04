function [ contact_list ] = feasibleContact( alpha, L )
n=size(alpha,1)+1;%picking 3 contact modules from n modules
g=jointsInHead(alpha,L);%get all the frames
%get the points
p=zeros(2,n+1);
for i=1:n+1
    p(:,i)=g{i}(1:2,3);
end

com=computeCOM(framesInHead(alpha,L));%compute COM position relative to the head
%3 for loops will do
contact_list=[];
for i=1:n-3
    for j=i+2:n-1
        %check if in
        if inpolygon(com(1),com(2),p(1,[i,i+1,i+2,j,j+1,j+2]),p(2,[i,i+1,i+2,j,j+1,j+2]))
            %put into the contact list
            contact_list=[contact_list,[i;i+1;j;j+1]];%#ok%all the contact modules
        else
            %throw

        end
    end
end



end


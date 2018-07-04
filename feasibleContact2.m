function [ contact_list ] = feasibleContact2( alpha, L, K )

n=size(alpha,1)+1;%picking 2 contact modules from n modules
g=jointsInHead(alpha,L);%get all the frames
%get the points
p=zeros(2,n+1);
for i=1:n+1
    p(:,i)=g{i}(1:2,3);
end

com=computeCOM(framesInHead(alpha,L));%compute COM position relative to the head
%generate all possible combinations
contact_list=nchoosek(1:n-1,K);
for r = size(contact_list,1):-1:1
    %check if in
    if inpolygon(com(1),com(2),p(1,[contact_list(r,:),contact_list(r,:)+1]),p(2,[contact_list(r,:),contact_list(r,:)+1]))
        %put into the contact list
        
    else
        %throw
        contact_list(r,:) = [];
    end
end

contact_list = [contact_list,contact_list+1]';

end


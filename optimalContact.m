function [ optimal_index ] = optimalContact( alpha0,alpha1, d_alpha, contact_list , desired_xi, dt, L, K)
desired_xi=desired_xi/norm(desired_xi);
freq=1/dt;
optimal_index=nan;%the index of optimal choice of contacts
n=size(alpha0,1)+1;
length=size(contact_list,2);%number of feasible contacts


dis_t=-inf;
for i=1:length%for each contact configuration
    %assume this contact, compute body velocity of the head
    activation=zeros(1, n);
    activation(contact_list(:,i))=1;
    
    %convert the body velocity of the head to the body velocity of the vc
    xi_v=xiVC(alpha0,alpha1,d_alpha,activation,freq,L, K);
    %measure cos distance
%     w=diag([1 1 10]);%objective weights
    eval=xi_v'*desired_xi-5*(xi_v(3)-desired_xi(3))^2;
%     eval=xi_v'*desired_xi/norm(xi_v);
    if eval>dis_t
        dis_t=eval;
        optimal_index=i;
    end
end


end


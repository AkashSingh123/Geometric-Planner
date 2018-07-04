function [ A_vc ] = computeVCLinearMap( alpha, contacts, L, K )
%generate the linear map from shape velocity to body velocity (measured
%relative to the VC)
A_h=computeLinearMap(alpha,L,contacts,K);
vc_in_head=vcInHead(alpha,L);
A_vc=adjoint(inv(vc_in_head))*A_h;

end


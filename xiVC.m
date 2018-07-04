function [ xi_vc ] = xiVC( alpha0, alpha1, d_alpha, contacts, freq, L, K,g_h,peg_spacing,vx,vy,r1,r2)
%given body velocity in head, computed body velocity in VC
vc_in_head0=vcInHead(alpha0,L);
vc_in_head1=vcInHead(alpha1,L);
xi_vc_in_head=logm(vc_in_head0\vc_in_head1)*freq;
xi=computeBodyVelocity(alpha0,d_alpha,L,contacts, K,g_h,peg_spacing,vx,vy,r1,r2);
%disp(xi);
xi_vc=adjoint(inv(vc_in_head0))*xi+unHat(xi_vc_in_head);

end


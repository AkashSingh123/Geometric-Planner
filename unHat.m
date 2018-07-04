function [ xi ] = unHat( xi_hat )
xi=[xi_hat(1,3);xi_hat(2,3);(-xi_hat(1,2)+xi_hat(2,1))/2];

end


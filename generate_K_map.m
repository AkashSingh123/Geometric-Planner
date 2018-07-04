function [K,K_height] = generate_K_map(x,y,peg_spacing,vx,vy,r1,r2)

x_min = -100;
x_max = 200;
y_min = -150;
y_max = 150;
k_compensate =2;


%
[peg_X,peg_Y] = meshgrid(x_min:peg_spacing:3,y_min:peg_spacing:y_max);
peg_X = peg_X(:);
 peg_Y = peg_Y(:);
% pegs_lineX = linspace(3.9822,100,14);
% pegs_lineY = -3.7326*ones(1,14);
% pegs_lineX2 = linspace(3.9822,100,14);
% pegs_lineY2 = -18.5046*ones(1,14);
% pegs_lineX3 = linspace(3.9822,100,14);
% pegs_lineY3 = -25.8906*ones(1,14);
% pegs_lineX4 = linspace(3.9822,100,14);
% pegs_lineY4 = -11.1186*ones(1,14);
% 
% peg_X = [peg_X(:);pegs_lineX';pegs_lineX2';pegs_lineX3';pegs_lineX4'];
% peg_Y = [peg_Y(:);pegs_lineY';pegs_lineY2';pegs_lineY3';pegs_lineY4'];
% % scatter(peg_X,peg_Y,10,'r','fill');
sigma_x = 0.5;
sigma_y = 0.5;
%

peg_X = [peg_X(:);r1];
peg_Y = [peg_Y(:);r2];
Gaussian_A = 90000;
K_height = 0;


for k = 1:length(peg_X)

           K_height = K_height + Gaussian_A*exp( - ((x-peg_X(k))^2/(2*sigma_x^2) + (y-peg_Y(k))^2/(2*sigma_y^2)));
%            A*exp( - (a*(x-m(i+21,1)).^2 + 2*c*(x-m(i+21,1)).*(y-p(1,j))+ b*(y-p(1,j)).^2)) ;

end
K_height = K_height+k_compensate;
K = [1,0,0;0,K_height,0;0,0,1];
end
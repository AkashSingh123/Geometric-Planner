%relation between density and displacement
function a = plotting(Distance_travelled_percycle)
lambda = 15.1568;
figure();hold on;axis equal;set(gcf,'color','w');

ylabel('Distance per cycle/wavelength','fontweight','bold','fontsize',20); % x-axis label
xlabel('peg spacing/wavelength','fontweight','bold','fontsize',20); % y-axis label
title('Wave Efficiency Vs Peg Spacing (UN = 2)','fontsize',20);

axis([0.2 1 0 1.2]);
daspect([1 3 1])
set(gca,'FontSize',20)

% Distance_travelled_percycle = [12.5806,   11.9836,   11.6344,  10.4078    8.8182    8.4773    7.6243   7.4396    5.3276    4.1402    1.5321    3.8656    5.8145;
%    12.5717   12.3166   11.4300   10.7304   11.8993    9.5574    8.1442    7.5929    4.7384    3.7908    0.6877    1.4064    6.9741;
%    12.5101   12.0390   11.2440   10.9545    8.6960    8.1898    7.8776    7.3793    3.8508    3.4306    0.6250    0.5664    7.0024;
%    12.5610   12.2379   11.7158   11.2977   11.2572    8.6469    8.3014    7.4760    5.5842    3.8382    1.0255    0.1852    6.4986;
%    12.5632   12.1219   11.3130   10.6440    9.6161    8.8789    8.6655    5.6529    4.1681    3.7600    1.0367    0.1121    6.9953;
%    12.5909   12.2961   11.2799   10.5888   11.3598    8.5285    9.0059    7.4353    6.2078    4.5452    4.8776    0.1085    4.0617;
%    12.5643   12.2454   11.5032   10.8560    9.4972    8.2269    8.3534    8.3277    6.3886    3.0962    4.8969    0.1040    3.7984;
%    12.5799   12.1420   11.5681   11.1030    9.6901    9.8323    8.1626    7.1333    6.3786    5.3504    3.7498    0.0991    6.0045;
%    12.5058   12.2675   11.2813   11.1724   10.6225    9.3226    8.0371    7.1540    5.7488    4.9115    5.5983    0.0952    2.4456;
%    12.5185   12.3917   11.3994   11.4021    9.2960    9.6746    6.5634    7.5588    5.7264    4.5761    4.8926    0.0926    4.8851;
%    12.5653   12.2399   11.5591   11.0184   11.6145    7.9104    8.4800    8.2755    4.7446    4.1562    2.3130    0.0909    6.6789;
%    12.5764   12.1399   11.5254   10.4200    9.3759    7.9333    9.2973    6.6986    5.8012    3.2249    4.6091    0.0892    5.7929;
%    12.5940   12.3684   11.2209   10.7520   11.5301    8.2503    8.4554    7.3949    4.7497    3.5252    5.2422    0.0866    6.1541;
%    12.5325   12.1966   11.4603   10.6377    8.9313    8.6013    7.6615    6.4261    6.1192    3.9701    6.9798    0.0822    5.0456;
%    12.4960   12.3976   11.6486   11.4364   11.7482    8.8495    7.9464    7.7690    7.2874    4.0101    5.2162    0.0752    5.1202;
%    12.5468   12.2504   11.2828   11.0322    8.9989    8.7926    8.0547    7.5882    6.3569    2.1388    4.4493    0.0659    4.8470;
%    12.5707   12.3720   11.3385   10.5270   11.7694    8.3710    7.9844    6.7438    6.8663    2.6586    5.1028    0.0553    3.3650;
%    12.5573   12.3048   11.5848   10.7169    9.2495    7.9611    7.8684    7.7300    5.9258    3.7511    5.4360    0.0445    3.5932;
%    12.5340   12.3595   11.4996   11.0935   11.8395    8.3087    8.4167    7.5087    6.6047    3.1726    5.2462    0.0348    4.6730;
%    12.3339   12.2111   11.2194   11.0892    8.6506    9.2767    7.7577    7.9226    7.0991    3.0595    5.2282    0.0266    5.7802;
%     9.6848   10.2874   11.2488   11.4530    9.9333   10.2121    9.1084    7.5525    5.8851    3.7794    6.1388    0.0204    5.6989]
Distance_travelled_percycle = Distance_travelled_percycle/lambda;
x = repmat((3:15)/lambda,21,1);

% for j=1:21
% 
% plot(x(:),Distance_travelled_percycle(j,:),'linewidth',2);
% 
% end


scatter(x(:),Distance_travelled_percycle(:));

end
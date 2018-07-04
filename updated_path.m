function [m,n]=updated_path(target_point_buffer)
xtra_path_x=[];
xtra_path_y=[];


% 
% target_point_buffer =[ 0   ,     0;
%          0    ,     0;
%          0    ,     0;
%          0    ,     0;
%          0    ,     0;
%          0    ,     0;
%     7.1828 , -16.3116;
%     7.1828 , -22.6976;
%     7.9829 , -23.4997;
%    13.5840 , -22.0961;
%    13.5840 , -17.0540;
%    17.0755 , -14.8268;
%    17.0755 , -12.7759;
%    22.4472 ,  -9.4865;
%    25.8044 , -13.0842;
%    29.8331 ,  -9.3837;
%    34.5333 , -12.9815;
%    37.2191 ,  -9.6921;
%    43.2622 , -12.4675;
%    44.6051 , -10.4116;
%    51.9911 , -11.5424];

for i=4:size(target_point_buffer,1)-1
    theta = atan2((target_point_buffer(i+1,2)-target_point_buffer(i,2)),(target_point_buffer(i+1,1)-target_point_buffer(i,1)));
    l=pdist2(target_point_buffer(i,1:2),target_point_buffer(i+1,1:2));
    l_adjst=1.8*l/7.3860;
    xtra_path_x = [xtra_path_x,linspace(target_point_buffer(i,1)+cos(theta)*l_adjst,target_point_buffer(i+1,1)-cos(theta)*l_adjst,15)];
    xtra_path_y = [xtra_path_y,linspace(target_point_buffer(i,2)+sin(theta)*l_adjst,target_point_buffer(i+1,2)-sin(theta)*l_adjst,15)]; 
end
    m=[xtra_path_x];
    n=[xtra_path_y];


end
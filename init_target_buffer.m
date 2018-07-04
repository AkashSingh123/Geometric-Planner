function init_tarbuf=init_target_buffer(A,g_h)

head_pos=[g_h(1,3),g_h(2,3)];
dist_matrix=pdist2(head_pos,A);

%tip =g_h*inv(F);
%tip_cord=[tip(1,3),tip(2,3)];
sorted_dist_matrix=sortrows(dist_matrix');
%disp(sorted_dist_matrix);
index=find(dist_matrix==sorted_dist_matrix(1));
target_point1=A(index,:);
index=find(dist_matrix==sorted_dist_matrix(2));
target_point2=A(index,:);
init_tarbuf=[target_point1;target_point2];

end
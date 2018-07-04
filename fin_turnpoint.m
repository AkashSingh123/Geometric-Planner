function turn_point=fin_turnpoint(vx,vy,g_h)
i=0;
k=1;
a=[vx(1,1:13)];%,vx(2,:)];
b=[vy(1,1:13)];%,vy(2,:)];
F=[eye(2),[2;0];0 0 1];
g_h=g_h*inv(F);
voronoi_set=[a;b];
head_pos=[g_h(1,3),g_h(2,3)];
c=pdist2(head_pos,voronoi_set');
sorted_matrix=sortrows(c');
l=voronoi_set';
while(k==1)
i=i+1;
b=find(c==sorted_matrix(i));
%disp(b);
if(l(b(1),1)<head_pos(1))
    turn_point=l(b(1),:);
    disp(turn_point);
   % scatter(turn_point(1),turn_point(2));
    k=0;
else
end
end
end
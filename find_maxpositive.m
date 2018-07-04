function num=find_maxpositive(a)
b=size(a,2);
k=1;

for r=b:-1:1
if(k==1)
    if(a(r)==1)
        num=r;
        k=0;
    end
end
end
end
    
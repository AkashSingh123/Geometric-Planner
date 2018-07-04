function [ c ] = contactCost( contacts0, contacts1 )
w=1;
if isempty(contacts0)
    c=0;
else
    c=w*sum(abs(contacts0(end,:)-contacts1(end,:)));
end

end


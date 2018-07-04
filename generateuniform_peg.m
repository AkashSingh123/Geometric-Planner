function uniform_pegs = generateuniform_peg();
for i= -20:20:60
m(i+21,:)=linspace(i,i,10);
n(i+21,:)=linspace(-40,40,10);

scatter(m(i+21,:),n(i+21,:),10,'r','fill');
end

function [ conf1 ] = forwardSim( conf0, alpha0, alpha1, d_alpha, contacts, dt, L, K ,g_h,peg_spacing,vx,vy,r1,r2)
%given a config_node, and a contact state expand, simulate resultant
%config_node
xi_vc=xiVC(alpha0,alpha1, d_alpha, contacts, 1/dt, L, K,g_h,peg_spacing,vx,vy,r1,r2);
factor=[1;1;1]*1;
dg=expm(xiHat(xi_vc.*factor*dt));
%expand new conf
conf1=conf0;
conf1.g=conf0.g*dg;
conf1.t=conf0.t+dt;
conf1.contacts=[conf0.contacts;contacts];
conf1.contact_cost=conf0.contact_cost+contactCost(conf0.contacts, contacts);
conf1.heuristic_cost=heuristicCost(conf1.g);
conf1.estimated_cost=conf1.contact_cost+conf1.heuristic_cost;

end
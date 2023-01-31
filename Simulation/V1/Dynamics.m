clear all;
syms Mw
syms Mp
syms Jw
syms Jp
syms Lp g R
syms theta(t) x(t) tau(t) t   %Çý¶¯ÂÖ½ÇÎ»ÒÆ
Vpx=diff(x,t)+0.5*Lp*diff(theta,t)*cos(theta);
Vpy=-0.5*diff(theta,t)*Lp*sin(theta);   
T1=0.5*Mw*diff(x,t)^2+0.5*Jw*diff(x,t)^2/R^2;
T2=0.5*Jp*diff(theta,t)^2+0.5*Mp*(Vpx^2+Vpy^2);
V2=0.5*Mp*g*Lp*cos(theta);
L=T1+T2-V2;
L=simplify(L);
eqn=functionalDerivative(L,[x;theta])==[-tau;0];
eqn=simplify(eqn);
eqn_latex=latex(eqn);











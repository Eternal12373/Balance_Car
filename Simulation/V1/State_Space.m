clear all;
syms Jw Mp R Mw Lp Jp F theta
Lp=0.3;
R=0.05;
Mw=0.5;
Mp=1;
Jp=1/3*Mp*Lp^2;
Jw=0.5*Mw*R^2;
a=2*(Jw+Mp*R^2+Mw*R^2);
b=Lp*Mp*R^2;
c=2*Mp*Lp;
d=Mp*Lp^2+4*Jp;
Qtemp=[a,b;c,d];
Qinv=inv(Qtemp);

Qinv*[2*R^2*F;2*Lp*Mp*9.8*theta]

A=[0 1 0 0
    0 0 -588/185 0
    0 0 0 1
    0 0 1372/37 0];
B=[0 28/37 0 -80/37]';
C=[1 0 0 0
     0 0 1 0];
 D=[0 0]';
 sys=ss(A,B,C,D);
 
% eig(A)
% Qc=ctrb(A,B);
% rank(Qc)
% Qb=obsv(A,C);
% rank(Qb)
%¿É¿Ø¿É²â

Q=[1 0 0 0;
    0 10000 0 0;
    0 0 80000 0;
    0 0 0 1];
R=0.1;
K=lqr(A,B,Q,R);
Ac=A-B*K;
x0=[0.1;0;0.1;0];
t=0:0.05:20;
u=zeros(size(t));
lsim(Ac,B,C,D,u,t,x0);
syslqr=ss(Ac,B,C,D);


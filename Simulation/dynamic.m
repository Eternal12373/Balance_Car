syms M  %��������
syms m  %����������
syms L   %�������ĺ��������
syms r   %�����ְ뾶
syms D  %���������־���
syms g  %�������ٶ�
syms theta(t) %pitch�Ƕ�
syms TL TR %��������������
syms xl(t) xr(t) %����������λ��
syms OL OR %������������ת���ٶ�
syms Jw %������ת������
syms Jp %����ת������
syms taul(t) taur(t)
syms tau(t)
x=0.5*(xl+xr);
vx=diff(x,t)+L*diff(theta,t)*cos(theta);
vy=L*diff(theta,t)*sin(theta);
T1=0.5*m*diff(xl,t)^2+0.5*m*diff(xr,t)^2+0.5*Jw*(diff(xl,t)/r)^2+0.5*Jw*(diff(xr,t)/r)^2;
T2=0.5*M*(vx^2+vy^2)+0.5*Jp*diff(theta,t)^2;
V2=M*g*L*cos(theta);
L=T1+T2-V2;
L=simplify(L);
eqn=functionalDerivative(L,[xl;xr;theta])==[-taul/r;-taur/r;-(taul+taur)]; %�޻�����
eqn=simplify(eqn);
eqn_latex=latex(eqn);





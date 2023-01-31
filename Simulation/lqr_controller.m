M=6;  %��������
m=1.5;  %����������
L=0.1;   %�������ĺ��������
r =0.074;  %�����ְ뾶
D=0.523;  %���������־���
g=9.8;  %�������ٶ�
Jw=2788.825e-6;%������ת������
Jp=153010.57e-6; %����ת������
Jy=182050.38e-6; %������y��ת������

a=r*(M+2*m+2*Jw/r^2);
b=M*r*L;
c=Jp+M*L^2;
d=M*g*L;
e=M*L;
f=1/(r*(m*D+Jw*D/r^2+2*Jy/D));

A23=-b*d/(a*c-b*e);
A43=a*d/(a*c-b*e);
B21=(c+b)/(a*c-b*e);
B22=(c+b)/(a*c-b*e);
B41=-(e+a)/(a*c-b*e);
B42=-(e+a)/(a*c-b*e);
B61=f;
B62=-f;

A=[0 1  0  0 0 0;
   0 0 A23 0 0 0;
   0 0  0  1 0 0;
   0 0 A43 0 0 0;
   0 0  0  0 0 1;
   0 0  0  0 0 0]  ;                         
B=[0 0;B21 B22;0 0;B41 B42;0 0;B61 B62];
C=[1 0 0 0 0 0   %λ��
     0 0 1 0 0 0];  %�Ƕ�
D=[1 0;0 1];
sys=ss(A,B,C,D);
Co=ctrb(A,B);

if(rank(Co)==6)
    disp('�ɿ�');
else
    disp('���ɿ�');
end

Q=[1 0 0 0 0 0;
    0 10000 0 0 0 0;
    0 0 80000 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 10000 0;
    0 0 0 0 0 0];
R=[1000 0;
      0 1000];
K=lqr(A,B,Q,R);
Ac=A-B*K;

x0=[0 0 0.34 0 0 0]';
t=0:0.01:50;
u=[zeros(size(t));zeros(size(t))];
lsim(Ac,B,C,D,u,t,x0);
syslqr=ss(Ac,B,C,D);
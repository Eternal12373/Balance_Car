syms M  %机体质量
syms m  %驱动轮质量
syms L   %机体质心和轮轴距离
syms r   %驱动轮半径
syms D  %左右驱动轮距离
syms g  %重力加速度
syms theta(t) %pitch角度
syms TL TR %左右驱动轮力矩
syms xl(t) xr(t) %左右驱动轮位移
syms OL OR %左右驱动轮旋转角速度
syms Jw %驱动轮转动惯量
syms Jp %机体转动惯量
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
eqn=functionalDerivative(L,[xl;xr;theta])==[-taul/r;-taur/r;-(taul+taur)]; %无滑动，
eqn=simplify(eqn);
eqn_latex=latex(eqn);





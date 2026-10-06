function  [th_new,d_new]=InvKinematic(J,yaw,th,d)
a=[400 250 0 0]';
al=[0 0 0 pi]';
% Reference Inverse Kinematics Problem /93
arcos_th2_new=acos((J(1)^2+J(2)^2- a(1)^2-a(2)^2)/(2*a(1)*a(2)));
if abs(th(2)-arcos_th2_new)<abs(th(2)+arcos_th2_new)
    th2_new=arcos_th2_new;
else th2_new=-arcos_th2_new;
end;

th1_new=atan2(J(2),J(1))-atan2(a(2)*sin(th2_new),a(1)+a(2)*cos(th2_new));

th4_new=-th1_new-th2_new+yaw;

d3_new=J(3)-d(1)-d(4);
th_new=[th1_new th2_new 0 th4_new];
d_new=[347.25 0 d3_new -30]';
